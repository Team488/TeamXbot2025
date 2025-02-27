package competition.subsystems.drive.commands;

import competition.subsystems.pose.PoseSubsystem;
import competition.subsystems.vision.AprilTagVisionSubsystemExtended;
import competition.subsystems.vision.CoprocessorCommunicationSubsystem;
import org.kobe.xbot.JClient.CachedSubscriber;
import org.kobe.xbot.JClient.XTablesClient;
import org.kobe.xbot.Utilities.Entities.XTableValues;
import xbot.common.logging.RobotAssertionManager;
import xbot.common.properties.PropertyFactory;
import xbot.common.subsystems.drive.BaseSwerveDriveSubsystem;
import xbot.common.subsystems.drive.SwervePointKinematics;
import xbot.common.subsystems.drive.SwerveSimpleTrajectoryMode;
import xbot.common.subsystems.drive.control_logic.HeadingModule;

import javax.inject.Inject;

public class SwerveBezierTrajectoryCommand extends SwerveBezierTrajectoryBase {

    private final CoprocessorCommunicationSubsystem coprocessorCommunicationSubsystem;
    private CachedSubscriber subscriber;
    private XTableValues.BezierCurves curves;

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
        this.logic.setVelocityMode(SwerveSimpleTrajectoryMode.GlobalKinematicsValue);
        this.logic.setGlobalKinematicValues(new SwervePointKinematics(3, 0, 0, 3));
        XTablesClient client = this.coprocessorCommunicationSubsystem.tryGetXTablesClient();
        if (client != null) {
            log.info("Retrieving drive trajectory");
            curves = client.getBezierCurves("bezier_path");
            if (curves != null && !curves.getCurvesList().isEmpty()) {
                log.info("Retrieved drive trajectory from XTABLES, proceeding to drive.");
                subscriber = new CachedSubscriber("bezier_path", client, 1, true);
                final XTableValues.TraversalOptions options = curves.hasOptions() ? curves.getOptions() : null;
                setSegmentedBezierCurve(curves, options);
                super.initialize();
            } else {
                log.warn("There was no drive trajectory found from XTABLES, cancelling.");
                cancel();
            }
        }

    }

    @Override
    public void execute() {
        if (subscriber != null && curves != null && subscriber.getAsBezierCurves(curves).equals(curves)) {
            super.execute();
        } else if(subscriber != null) {
            curves = subscriber.getAsBezierCurves(null);
            if (curves != null && !curves.getCurvesList().isEmpty()) {
                log.info("There was a new drive trajectory, updating to drive.");
                drive.stop();
                final XTableValues.TraversalOptions options = curves.hasOptions() ? curves.getOptions() : null;
                setSegmentedBezierCurve(curves, options);
                super.initialize();
            }
        } else {
            log.warn("No drive trajectory found from XTABLES, cancelling.");
            cancel();
        }
    }

    @Override
    public boolean isFinished() {
       return getAlternativeIsFinishedSupplier().get();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        if (interrupted) {
            log.warn("Command interrupted");
        }
        this.subscriber.unsubscribe();
    }

}
