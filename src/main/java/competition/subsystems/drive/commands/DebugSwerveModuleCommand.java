package competition.subsystems.drive.commands;

import competition.operator_interface.OperatorInterface;
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
    }

    @Override
    public void initialize() {
        log.info("Initializing");
    }

    @Override
    public void execute() {
        double drivePower = oi.programmerGamepad.getLeftStickY();
        double turnPower = oi.programmerGamepad.getRightStickX();

        drive.controlOnlyActiveSwerveModuleSubsystem(drivePower, turnPower);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
