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
        // Use a generous deadband to prevent surprise movements
        double drivePower = MathUtil.applyDeadband(oi.programmerGamepad.getLeftStickY(), 0.15);
        double turnPower = MathUtil.applyDeadband(oi.programmerGamepad.getRightStickX(), 0.15);

        drive.controlOnlyActiveSwerveModuleSubsystem(drivePower, turnPower);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
