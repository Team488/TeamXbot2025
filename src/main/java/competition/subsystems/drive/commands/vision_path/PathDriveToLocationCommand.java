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
import xbot.common.controls.sensors.XTimer;
import xbot.common.logging.RobotAssertionManager;
import xbot.common.properties.PropertyFactory;
import xbot.common.subsystems.drive.BaseSwerveDriveSubsystem;
import xbot.common.subsystems.drive.control_logic.HeadingModule;
import xbot.common.trajectory.XbotSwervePoint;

import javax.inject.Inject;
import java.util.List;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

import static edu.wpi.first.units.Units.Inches;

public class PathDriveToLocationCommand extends SwerveBezierTrajectoryBase {
    Pose2d target;

    private final VisionCoprocessorCommander commander;
    private XTableValues.TraversalOptions traversalOptions;
    private Distance safeDistance = Inches.of(10);
    private final ReefRoutingCircle routingCircle;
    private XTableValues.AdditionalArguments additionalArguments;

    private CoprocessorCommunicationSubsystem coprocessor;
    protected AtomicBoolean failed = new AtomicBoolean(false);
    protected AtomicReference<XTableValues.BezierCurves> curves = new AtomicReference<>(null);
    private boolean lock = false;

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
        curves.set(null);
        lock = false;
        failed.set(false);
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
            commander.requestBezierPathWithOptionsAsync(
                    message
                            .build(),
                    500, TimeUnit.MILLISECONDS, (response) -> {
                        curves.set(response);
                        failed.set(false);
                        coprocessor.setCoprocessorHealthy(true);
                        XTablesClient client = this.coprocessor.getXTablesManager().getOrNull();
                        if (client != null) {
                            log.info("Logged bezier curves onto XTABLES.");
                            client.putBezierCurves("bezier_path", response);
                        }
                    }, (err) -> {
                        coprocessor.setCoprocessorHealthy(false);
                        failed.set(true);
                        curves.set(null);
                        log.warn("Path drive failed with an error! Reason: {}", err.getMessage());
                    }); // When should it give up and return
        } else {
            // Use backup Point To Point if settings allow to.
            pointToPoint();
        }
    }

    public XTableValues.BezierCurves getCurves() {
        return curves.get();
    }

    private void pointToPoint() {
        List<XbotSwervePoint> swervePoints =
                routingCircle.generateSwervePoints(pose.getCurrentPose2d(), target);
        super.logic.setKeyPoints(swervePoints);
    }

    @Override
    public void execute() {
        if(!lock && curves.get() != null && !failed.get()) {
            XTableValues.BezierCurves c = curves.get();
            this.setSegmentedBezierCurve(c, c.getOptions());
            super.initialize();
            lock = true;
            return;
        }
        if(curves != null && curves.get() != null && !failed.get()) {
            super.execute();
        }
    }

    @Override
    public boolean isFinished() {
        return !coprocessor.getIsCoprocessorHealthy() || super.isFinished();
    }
}
