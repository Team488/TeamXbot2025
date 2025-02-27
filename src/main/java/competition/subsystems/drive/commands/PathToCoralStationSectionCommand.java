package competition.subsystems.drive.commands;

import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.oracle.ReefRoutingCircle;
import competition.subsystems.pose.Landmarks;
import competition.subsystems.pose.PoseSubsystem;
import competition.subsystems.vision.CoprocessorCommunicationSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.kobe.xbot.Utilities.Entities.XTableValues;
import org.kobe.xbot.Utilities.VisionCoprocessorCommander;
import xbot.common.logging.RobotAssertionManager;
import xbot.common.properties.PropertyFactory;
import xbot.common.subsystems.drive.control_logic.HeadingModule;
import xbot.common.trajectory.XbotSwervePoint;

import javax.inject.Inject;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;
import java.util.stream.Collectors;

public class PathToCoralStationSectionCommand extends SwerveBezierTrajectoryBase {

    Pose2d targetCoralStationSection;
    private final VisionCoprocessorCommander commander;


    private static boolean useBackupPointToPoint = false;
    private final ReefRoutingCircle routingCircle;


    @Inject
    public PathToCoralStationSectionCommand(DriveSubsystem drive, PoseSubsystem pose,
                                            PropertyFactory pf, HeadingModule.HeadingModuleFactory headingModuleFactory,
                                            RobotAssertionManager robotAssertionManager, CoprocessorCommunicationSubsystem coprocessorCommunicationSubsystem) {
        super(drive, pose, pf, headingModuleFactory, robotAssertionManager, coprocessorCommunicationSubsystem);
        this.commander = coprocessorCommunicationSubsystem.getOrinVisionCoprocessorCommander();
        Translation2d center = Landmarks.BlueCenterOfReef.getTranslation();
        routingCircle = new ReefRoutingCircle(center, 2);
    }

    public void setTargetCoralStationSection(Landmarks.CoralStation station, Landmarks.CoralStationSection section) {
        this.targetCoralStationSection = Landmarks.getCoralStationSectionPose(station, section);
    }

    public static void setUseBackupPointToPoint(boolean useBackupPointToPoint) {
        PathToCoralStationSectionCommand.useBackupPointToPoint = useBackupPointToPoint;
    }

    @Override
    public void initialize() {
        log.info("Initializing");
        Pose2d startingPose = pose.getCurrentPose2d();
        XTableValues.BezierCurves curves = null;
        if (!useBackupPointToPoint) {
            curves = commander
                    .requestBezierPathWithOptions(XTableValues.RequestVisionCoprocessorMessage.newBuilder()
                            .setStart(XTableValues.ControlPoint.newBuilder()
                                    .setY(startingPose.getY()) // Set Pose2d Y value.
                                    .setX(startingPose.getX()) // Set Pose2d X value.
                                    .build())
                            .setEnd(XTableValues.ControlPoint.newBuilder()
                                    .setX(targetCoralStationSection.getX()) // Set goal Pose2d X value.
                                    .setY(targetCoralStationSection.getY()) // Set goal Pose2d Y value.
                                    .setRotationDegrees(targetCoralStationSection.getRotation().getDegrees()) // Set goal rotation.
                                    .build())
                            .setSafeDistanceInches(35) // Will stay an EXTRA 40 inches away (recommended current no DeadWheels)
                            .setOptions(XTableValues.TraversalOptions.newBuilder() // Create a new option builder.
                                    .setMetersPerSecond(2)
                                    .setAccelerationMetersPerSecond(1)
                                    .setFinalRotationDegrees(targetCoralStationSection.getRotation().getDegrees()) // What should the final rotation be?
                                    .setFinalRotationTurnSpeedFactor(40) // How fast should it turn back to final rotation (2x)?
                                    .build())
                            .build(), 1000, TimeUnit.MILLISECONDS); // When should it give up and return null for any reason?
        }
        if (curves == null) {
            useBackupPointToPoint = true;
            log.info("No curves returned from vision coprocessor within timeout!");
            pointToPoint();
        } else {
            this.setSegmentedBezierCurve(curves, curves.getOptions());
        }
        super.initialize();
    }

    private void pointToPoint() {
        List<XbotSwervePoint> swervePoints = routingCircle.generateSwervePoints(pose.getCurrentPose2d(), targetCoralStationSection);
        logic.setKeyPoints(swervePoints);
    }
}
