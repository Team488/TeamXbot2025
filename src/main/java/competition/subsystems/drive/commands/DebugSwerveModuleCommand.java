package competition.subsystems.drive.commands;

import competition.operator_interface.OperatorInterface;
import edu.wpi.first.math.MathUtil;
import xbot.common.command.BaseCommand;
import xbot.common.subsystems.drive.BaseSwerveDriveSubsystem;

import javax.inject.Inject;

public class DebugSwerveModuleCommand extends BaseCommand {

    final BaseSwerveDriveSubsystem drive;
    final OperatorInterface oi;

    @Inject
    public DebugSwerveModuleCommand(BaseSwerveDriveSubsystem drive, OperatorInterface oi) {
        this.drive = drive;
        this.oi = oi;
        this.addRequirements(drive);
    }

    @Override
    public void initialize() {
        log.info("Initializing");
    }

    @Override
    public void execute() {
        double drivePower = MathUtil.applyDeadband(oi.superstructureGamepad.getLeftStickY(), oi.getDriverGamepadTypicalDeadband());
        double turnPower = MathUtil.applyDeadband(oi.superstructureGamepad.getRightStickX(), oi.getDriverGamepadTypicalDeadband());

        drive.controlOnlyActiveSwerveModuleSubsystem(drivePower, turnPower);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
