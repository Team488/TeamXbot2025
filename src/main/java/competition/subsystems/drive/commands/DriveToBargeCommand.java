package competition.subsystems.drive.commands;

import competition.subsystems.drive.commands.vision_path.PathDriveToLocationCommand;
import competition.subsystems.pose.PoseSubsystem;
import competition.subsystems.vision.CoprocessorCommunicationSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import xbot.common.logging.RobotAssertionManager;
import xbot.common.properties.DoubleProperty;
import xbot.common.properties.PropertyFactory;
import xbot.common.subsystems.drive.BaseSwerveDriveSubsystem;
import xbot.common.subsystems.drive.control_logic.HeadingModule;

import javax.inject.Inject;

public class DriveToBargeCommand extends PathDriveToLocationCommand {

    public DoubleProperty midFieldX;
    public DoubleProperty distanceFromBarge;

    @Inject
    public DriveToBargeCommand(BaseSwerveDriveSubsystem drive,
                               PoseSubsystem pose, PropertyFactory pf,
                               HeadingModule.HeadingModuleFactory headingModuleFactory,
                               RobotAssertionManager robotAssertionManager,
                               CoprocessorCommunicationSubsystem coprocessorCommunicationSubsystem) {
        super(drive, pose, pf, headingModuleFactory, robotAssertionManager, coprocessorCommunicationSubsystem);
        distanceFromBarge = pf.createPersistentProperty("distanceFromBarge-m", 1);
        midFieldX = pf.createPersistentProperty("midFieldX-m", 8.75665);
    }

    @Override
    public void initialize() {
        boolean blue = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red;
        Pose2d goal = new Pose2d(
                blue? midFieldX.get() - distanceFromBarge.get() : midFieldX.get() + distanceFromBarge.get(),
                pose.getCurrentPose2d().getY(),
                blue? Rotation2d.fromDegrees(180) : Rotation2d.fromDegrees(0)
        );
        super.setTarget(goal);
        super.initialize();
    }
}
