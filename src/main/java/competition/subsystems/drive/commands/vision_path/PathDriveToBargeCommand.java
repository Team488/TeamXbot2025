package competition.subsystems.drive.commands.vision_path;

import competition.subsystems.pose.PoseSubsystem;
import competition.subsystems.vision.CoprocessorCommunicationSubsystem;
import org.kobe.xbot.Utilities.Entities.XTableValues;
import xbot.common.logging.RobotAssertionManager;
import xbot.common.properties.DoubleProperty;
import xbot.common.properties.PropertyFactory;
import xbot.common.subsystems.drive.BaseSwerveDriveSubsystem;
import xbot.common.subsystems.drive.control_logic.HeadingModule;

import javax.inject.Inject;

public class PathDriveToBargeCommand extends PathDriveToLocationCommand {

    public DoubleProperty midFieldX;
    public DoubleProperty distanceFromBarge;

    @Inject
    public PathDriveToBargeCommand(BaseSwerveDriveSubsystem drive,
                                   PoseSubsystem pose, PropertyFactory pf,
                                   HeadingModule.HeadingModuleFactory headingModuleFactory,
                                   RobotAssertionManager robotAssertionManager,
                                   CoprocessorCommunicationSubsystem coprocessorCommunicationSubsystem) {
        super(drive, pose, pf, headingModuleFactory, robotAssertionManager, coprocessorCommunicationSubsystem);
        pf.setPrefix("PathDriveToBargeCommand");
    }

    @Override
    public void initialize() {
        if(coprocessor.isBargePathConfident(pose)) {
            XTableValues.BezierCurves path = coprocessor.getLastBargePath();
            super.setOverriddenPath(path);
            super.setOptions(path.getOptions());
            super.logic.setPrioritizeRotationIfCloseToGoal(true);
            super.logic.setRotationPrioritizationScaleback(0.5);
            super.setInstantRotation(true);
            super.initialize();
        } else {
            log.warn("Barge path could not be determined. Cancelling.");
            cancel();
        }
    }
}
