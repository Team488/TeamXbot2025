package competition.subsystems.drive.commands.vision_path;

import competition.subsystems.pose.Landmarks;
import competition.subsystems.pose.PoseSubsystem;
import competition.subsystems.vision.CoprocessorCommunicationSubsystem;
import org.kobe.xbot.Utilities.Entities.XTableValues;
import xbot.common.logging.RobotAssertionManager;
import xbot.common.properties.DoubleProperty;
import xbot.common.properties.PropertyFactory;
import xbot.common.subsystems.drive.BaseSwerveDriveSubsystem;
import xbot.common.subsystems.drive.control_logic.HeadingModule;

import javax.inject.Inject;

import static edu.wpi.first.units.Units.Inches;

public class PathDriveToNearestReefBranch extends PathDriveToLocationCommand {

    public Landmarks.Branch branch;

    @Inject
    public PathDriveToNearestReefBranch(BaseSwerveDriveSubsystem drive,
                                        PoseSubsystem pose, PropertyFactory pf,
                                        HeadingModule.HeadingModuleFactory headingModuleFactory,
                                        RobotAssertionManager robotAssertionManager,
                                        CoprocessorCommunicationSubsystem coprocessorCommunicationSubsystem) {
        super(drive, pose, pf, headingModuleFactory, robotAssertionManager, coprocessorCommunicationSubsystem);
        pf.setPrefix("PathDriveToBargeCommand");
    }

    public PathDriveToNearestReefBranch setBranch(Landmarks.Branch branch) {
        this.branch = branch;
        return this;
    }

    @Override
    public void initialize() {
        super.setDistanceToGoalAndFinish(Inches.of(0));
        if (branch != null) {
            if (branch.equals(Landmarks.Branch.A)) {
                if (coprocessor.isNearestReefBranchAPathConfident(pose)) {
                    XTableValues.BezierCurves path = coprocessor.getPathToNearestReefBranchACurves();
                    super.setOverriddenPath(path);
                    super.setOptions(path.getOptions());
                    super.logic.setPrioritizeRotationIfCloseToGoal(true);
                    super.setInstantRotation(true);
                    super.logic.setRotationPrioritizationScaleback(0.01);
                    super.setInstantRotation(true);
                    super.initialize();
                } else {
                    log.warn("path could not be determined. Cancelling.");
                    cancel();
                }
            } else    if (branch.equals(Landmarks.Branch.B)) {
                if (coprocessor.isNearestReefBranchBPathConfident(pose)) {
                    XTableValues.BezierCurves path = coprocessor.getPathToNearestReefBranchBCurves();
                    super.setOverriddenPath(path);
                    super.setOptions(path.getOptions());
                    super.logic.setPrioritizeRotationIfCloseToGoal(true);
                    super.setInstantRotation(true);
                    super.logic.setRotationPrioritizationScaleback(0.01);
                    super.setInstantRotation(true);
                    super.initialize();
                } else {
                    log.warn("path could not be determined. Cancelling.");
                    cancel();
                }
            }
        }
    }
}
