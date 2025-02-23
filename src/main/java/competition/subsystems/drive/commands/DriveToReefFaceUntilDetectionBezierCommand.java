package competition.subsystems.drive.commands;

import competition.subsystems.pose.Cameras;
import competition.subsystems.pose.Landmarks;
import competition.subsystems.pose.PoseSubsystem;
import competition.subsystems.vision.AprilTagVisionSubsystemExtended;
import competition.subsystems.vision.CoprocessorCommunicationSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import org.kobe.xbot.Utilities.Entities.VisionCoprocessor;
import org.kobe.xbot.Utilities.Entities.XTableValues;
import org.kobe.xbot.Utilities.VisionCoprocessorCommander;
import xbot.common.logging.RobotAssertionManager;
import xbot.common.properties.PropertyFactory;
import xbot.common.subsystems.drive.BaseSwerveDriveSubsystem;
import xbot.common.subsystems.drive.control_logic.HeadingModule;

import javax.inject.Inject;
import java.util.concurrent.TimeUnit;

public class DriveToReefFaceUntilDetectionBezierCommand extends SwerveBezierTrajectoryCommand {

    Pose2d targetReefFacePose;
    AprilTagVisionSubsystemExtended aprilTagVisionSubsystem;
    private Cameras camera = Cameras.FRONT_LEFT_CAMERA;

    private double minimumDistanceMeters = 3;
    private final VisionCoprocessorCommander commander;

    @Inject
    public DriveToReefFaceUntilDetectionBezierCommand(BaseSwerveDriveSubsystem drive, PoseSubsystem pose,
                                                      PropertyFactory pf,
                                                      HeadingModule.HeadingModuleFactory headingModuleFactory,
                                                      AprilTagVisionSubsystemExtended aprilTagVisionSubsystem,
                                                      RobotAssertionManager robotAssertionManager,
                                                      CoprocessorCommunicationSubsystem coprocessorCommunicationSubsystem) {
        super(drive, pose, pf, headingModuleFactory, robotAssertionManager, coprocessorCommunicationSubsystem);
        this.aprilTagVisionSubsystem = aprilTagVisionSubsystem;
        this.commander = new VisionCoprocessorCommander(VisionCoprocessor.LOCALHOST); // Connect to ORIN-3

    }

    public DriveToReefFaceUntilDetectionBezierCommand setTargetReefFacePose(Landmarks.ReefFace targetReefFace) {
        this.targetReefFacePose = Landmarks.getReefFacePose(targetReefFace);
        return this;
    }

    @Override
    public void initialize() {
        log.info("Initializing");
        Pose2d startingPose = pose.getCurrentPose2d();
        XTableValues.BezierCurves curves = commander
                .requestBezierPathWithOptions(XTableValues.RequestVisionCoprocessorMessage.newBuilder()
                        .setStart(XTableValues.ControlPoint.newBuilder()
                                .setY(startingPose.getY()) // Set Pose2d Y value.
                                .setX(startingPose.getX()) // Set Pose2d X value.
                                .build())
                        .setEnd(XTableValues.ControlPoint.newBuilder()
                                .setX(targetReefFacePose.getX()) // Set goal Pose2d X value.
                                .setY(targetReefFacePose.getY()) // Set goal Pose2d Y value.
                                .setRotationDegrees(targetReefFacePose.getRotation().getDegrees()) // Set goal rotation.
                                .build())
                        .setSafeDistanceInches(15) // Will stay an EXTRA 40 inches away (recommended current no DeadWheels)
                        .setOptions(XTableValues.TraversalOptions.newBuilder() // Create a new option builder.
                                .setMetersPerSecond(1000)
                                .setAccelerationMetersPerSecond(1000)
                                .setFinalRotationDegrees(targetReefFacePose.getRotation().getDegrees()) // What should the final rotation be?
                                .setFinalRotationTurnSpeedFactor(40) // How fast should it turn back to final rotation (2x)?
                                .build())
                        .build(), 5, TimeUnit.SECONDS); // When should it give up and return null for any reason?
        if(curves == null) {
            log.info("No curves returned from vision coprocessor within timeout!");
            cancel();
            return;
        }
        this.setSegmentedBezierCurve(curves, curves.getOptions());

        super.initialize();
    }

    @Override
    public boolean isFinished() {
        return (aprilTagVisionSubsystem.doesCameraBestObservationHaveAprilTagId(camera.getIndex(),
                aprilTagVisionSubsystem.getTargetAprilTagID(targetReefFacePose)) && pose.getCurrentPose2d().getTranslation().getDistance(targetReefFacePose.getTranslation()) <= minimumDistanceMeters )
                || logic.recommendIsFinished(pose.getCurrentPose2d(), drive.getPositionalPid(), headingModule) || super.isFinished();
    }



    public void setAprilTagCamera(Cameras camera) {
        this.camera = camera;
    }
}
