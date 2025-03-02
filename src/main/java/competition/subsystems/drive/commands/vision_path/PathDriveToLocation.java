package competition.subsystems.drive.commands.vision_path;

import competition.subsystems.oracle.ReefRoutingCircle;
import competition.subsystems.pose.Landmarks;
import competition.subsystems.pose.PoseSubsystem;
import competition.subsystems.vision.AprilTagVisionSubsystemExtended;
import competition.subsystems.vision.CoprocessorCommunicationSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
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

public class PathDriveToLocation extends SwerveBezierTrajectoryBase {
    Pose2d target;

    private final VisionCoprocessorCommander commander;

    private static boolean useBackupPointToPoint = false;

    private XTableValues.TraversalOptions traversalOptions;
    private int safeInches = 10;
    private final ReefRoutingCircle routingCircle;
    private XTableValues.AdditionalArguments additionalArguments;

    public XTableValues.BezierCurves curves = null;
    private CoprocessorCommunicationSubsystem coprocessor;

    @Inject
    public PathDriveToLocation(BaseSwerveDriveSubsystem drive, PoseSubsystem pose,
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

    public PathDriveToLocation setTarget(Pose2d target) {
        this.target = target;
        return this;
    }

    public PathDriveToLocation setOptions(
            XTableValues.TraversalOptions traversalOptions) {
        this.traversalOptions = traversalOptions;
        return this;
    }

    public PathDriveToLocation setSafeInches(int safeInches) {
        this.safeInches = safeInches;
        return this;
    }

    public PathDriveToLocation setAdditionalArguments(XTableValues.AdditionalArguments additionalArguments) {
        this.additionalArguments = additionalArguments;
        return this;
    }

    public static void setUseBackupPointToPoint(boolean useBackupPointToPoint) {
        PathDriveToLocation.useBackupPointToPoint = useBackupPointToPoint;
    }

    @Override
    public void initialize() {
        log.info("Initializing");
        Pose2d startingPose = pose.getCurrentPose2d();
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
                .setSafeDistanceInches(safeInches);
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
            useBackupPointToPoint = true;
            log.warn("No curves returned from vision coprocessor within timeout! "
                    + "Using P2P from now on.");
        } else {

            this.setSegmentedBezierCurve(curves, curves.getOptions());
            XTablesClient client = this.coprocessor.getXTablesManager().getOrNull();
            if (client != null) {
                client.putBezierCurves("bezier_path", curves);
            }
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
