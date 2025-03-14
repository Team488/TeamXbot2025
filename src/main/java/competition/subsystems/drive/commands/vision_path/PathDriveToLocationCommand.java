package competition.subsystems.drive.commands.vision_path;

import competition.subsystems.oracle.ReefRoutingCircle;
import competition.subsystems.pose.Landmarks;
import competition.subsystems.pose.PoseSubsystem;
import competition.subsystems.vision.AprilTagVisionSubsystemExtended;
import competition.subsystems.vision.CoprocessorCommunicationSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import org.kobe.xbot.JClient.XTablesClient;
import org.kobe.xbot.Utilities.Entities.XTableValues;
import org.kobe.xbot.Utilities.VisionCoprocessorCommander;
import xbot.common.logging.RobotAssertionManager;
import xbot.common.properties.PropertyFactory;
import xbot.common.subsystems.drive.BaseSwerveDriveSubsystem;
import xbot.common.subsystems.drive.control_logic.HeadingModule;
import xbot.common.trajectory.XbotSwervePoint;

import javax.inject.Inject;
import java.util.List;
import java.util.concurrent.TimeUnit;

import static edu.wpi.first.units.Units.Inches;

public class PathDriveToLocationCommand extends SwerveBezierTrajectoryBase {
    Pose2d target;

    private final VisionCoprocessorCommander commander;
    private XTableValues.TraversalOptions traversalOptions;
    private Distance safeDistance = Inches.of(10);
    private final ReefRoutingCircle routingCircle;
    private XTableValues.AdditionalArguments additionalArguments;

    public XTableValues.BezierCurves curves = null;
    private CoprocessorCommunicationSubsystem coprocessor;

    @Inject
    public PathDriveToLocationCommand(BaseSwerveDriveSubsystem drive, PoseSubsystem pose,
                                      PropertyFactory pf,
                                      HeadingModule.HeadingModuleFactory headingModuleFactory,
                                      AprilTagVisionSubsystemExtended aprilTagVisionSubsystem,
                                      RobotAssertionManager robotAssertionManager,
                                      CoprocessorCommunicationSubsystem coprocessorCommunicationSubsystem) {
        super(drive, pose, pf, headingModuleFactory, robotAssertionManager,
                coprocessorCommunicationSubsystem);
        this.commander =
                coprocessorCommunicationSubsystem.getOrinVisionCoprocessorCommander();
        this.coprocessor = coprocessorCommunicationSubsystem;
        this.traversalOptions = XTableValues.TraversalOptions.newBuilder().build();
        Translation2d center = Landmarks.BlueCenterOfReef.getTranslation();
        routingCircle = new ReefRoutingCircle(center, 2);
    }

    public PathDriveToLocationCommand setTarget(Pose2d target) {
        this.target = target;
        return this;
    }

    public PathDriveToLocationCommand setOptions(
            XTableValues.TraversalOptions traversalOptions) {
        this.traversalOptions = traversalOptions;
        return this;
    }

    public PathDriveToLocationCommand setSafeDistance(Distance safeDistance) {
        this.safeDistance = safeDistance;
        return this;
    }

    public PathDriveToLocationCommand setAdditionalArguments(XTableValues.AdditionalArguments additionalArguments) {
        this.additionalArguments = additionalArguments;
        return this;
    }
    @Override
    public void initialize() {
        log.info("Initializing");
        Pose2d startingPose = pose.getCurrentPose2d();
        if(!coprocessor.isUseBackupPointToPointForPathplanning()) {
            curves = null;
            XTableValues.RequestVisionCoprocessorMessage.Builder message = XTableValues.RequestVisionCoprocessorMessage.newBuilder()
                    .setStart(XTableValues.ControlPoint.newBuilder()
                            .setY(startingPose.getY()) // Set Pose2d Y value.
                            .setX(startingPose.getX()) // Set Pose2d X value.
                            .build())
                    .setEnd(XTableValues.ControlPoint.newBuilder()
                            .setX(target.getX()) // Set goal Pose2d X value.
                            .setY(target.getY()) // Set goal Pose2d Y value.
                            .build())
                    .setSafeDistanceInches(safeDistance.in(Inches));
            if (traversalOptions != null) {
                message.setOptions(traversalOptions);
            }
            if (additionalArguments != null) {
                message.setArguments(additionalArguments);
            }
            curves = commander.requestBezierPathWithOptions(
                    message
                            .build(),
                    3000, TimeUnit.MILLISECONDS); // When should it give up and return
            // null for any reason?

            if (curves == null) {
                log.warn("There was not any curves responded from the coprocessor ORIN!");
//                cancel();

                return;
            } else {
                this.setSegmentedBezierCurve(curves, curves.getOptions());
                XTablesClient client = this.coprocessor.getXTablesManager().getOrNull();
                if (client != null) {
                    log.info("Logged bezier curves onto XTABLES.");
                    client.putBezierCurves("bezier_path", curves);
                }
            }
        } else {
            // Use backup Point To Point if settings allow to.
            pointToPoint();
        }
        super.initialize();
    }

    public XTableValues.BezierCurves getCurves() {
        return curves;
    }

    private void pointToPoint() {
        List<XbotSwervePoint> swervePoints =
                routingCircle.generateSwervePoints(pose.getCurrentPose2d(), target);
        super.logic.setKeyPoints(swervePoints);
    }
}
