package competition.subsystems.drive.commands;

import competition.subsystems.pose.Cameras;
import competition.subsystems.pose.Landmarks;
import competition.subsystems.pose.PoseSubsystem;
import competition.subsystems.vision.AprilTagVisionSubsystemExtended;
import competition.subsystems.vision.CoprocessorCommunicationSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import org.kobe.xbot.JClient.XTablesClient;
import org.kobe.xbot.Utilities.Entities.VisionCoprocessor;
import org.kobe.xbot.Utilities.Entities.XTableValues;
import org.kobe.xbot.Utilities.VisionCoprocessorCommander;
import xbot.common.logging.RobotAssertionManager;
import xbot.common.properties.PropertyFactory;
import xbot.common.subsystems.drive.BaseSwerveDriveSubsystem;
import xbot.common.subsystems.drive.SwerveSimpleTrajectoryMode;
import xbot.common.subsystems.drive.control_logic.HeadingModule;

import javax.inject.Inject;
import java.util.concurrent.TimeUnit;

public class SwerveBezierTrajectoryCommand extends SwerveBezierTrajectoryBase {

    private final CoprocessorCommunicationSubsystem coprocessorCommunicationSubsystem;

    @Inject
    public SwerveBezierTrajectoryCommand(BaseSwerveDriveSubsystem drive, PoseSubsystem pose,
                                         PropertyFactory pf,
                                         HeadingModule.HeadingModuleFactory headingModuleFactory,
                                         AprilTagVisionSubsystemExtended aprilTagVisionSubsystem,
                                         RobotAssertionManager robotAssertionManager,
                                         CoprocessorCommunicationSubsystem coprocessorCommunicationSubsystem) {
        super(drive, pose, pf, headingModuleFactory, robotAssertionManager, coprocessorCommunicationSubsystem);
        this.coprocessorCommunicationSubsystem = coprocessorCommunicationSubsystem;

    }

    @Override
    public void initialize() {
        this.logic.setVelocityMode(SwerveSimpleTrajectoryMode.ConstantVelocity);
        XTablesClient client = this.coprocessorCommunicationSubsystem.tryGetXTablesClient();
        if (client != null) {
            XTableValues.BezierCurves curves = client.getBezierCurves("bezier_path");
            if (curves != null && !curves.getCurvesList().isEmpty()) {
                final XTableValues.TraversalOptions options = curves.hasOptions() ? curves.getOptions() : null;
                setSegmentedBezierCurve(curves, options);
            }
        }
        super.initialize();
    }


}
