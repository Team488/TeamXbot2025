package competition.subsystems.drive.commands;

import competition.subsystems.drive.DriveSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import xbot.common.command.BaseCommand;

import javax.inject.Inject;


import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;

public class CalibrateDriveCommand extends BaseCommand {
    DriveSubsystem drive;
    Timer timer = new Timer();
    double power = 0.1;
    double frontLeftModuleStartPosition;
    double frontRightModuleStartPosition;
    double rearLeftModuleStartPosition;
    double rearRightModuleStartPosition;

    @Inject
    public CalibrateDriveCommand(DriveSubsystem drive) {
        this.drive = drive;
        this.addRequirements(drive);
    }

    @Override
    public void initialize() {
        log.info("Initializing");
        this.timer.restart();
        drive.setTargetSwerveStates(new SwerveModuleState(0,new Rotation2d(0)));

        // log starting drive motor positions
        frontLeftModuleStartPosition = drive.getFrontLeftSwerveModuleSubsystem().getDriveSubsystem()
                .getMotorController().getPosition().in(Degrees) / 360;
        frontRightModuleStartPosition = drive.getFrontRightSwerveModuleSubsystem()
                .getDriveSubsystem().getMotorController().getPosition().in(Degree) / 360;
        rearLeftModuleStartPosition = drive.getRearLeftSwerveModuleSubsystem()
                .getDriveSubsystem().getMotorController().getPosition().in(Degree) / 360;
        rearRightModuleStartPosition = drive.getRearRightSwerveModuleSubsystem()
                .getDriveSubsystem().getMotorController().getPosition().in(Degree) / 360;

        aKitLog.record("frontLeftModuleStartPosition", frontLeftModuleStartPosition);
        aKitLog.record("frontRightModuleStartPosition", frontRightModuleStartPosition);
        aKitLog.record("rearLeftModuleStartPosition", rearLeftModuleStartPosition);
        aKitLog.record("rearRightModuleStartPosition", rearRightModuleStartPosition);
    }

    @Override
    public void execute() {
        // wait for modules to be zeroed
        if (timer.hasElapsed(2)) {
            drive.setTargetSwerveStates(new SwerveModuleState(power, new Rotation2d(0)));
            double frontLeftModuleDeltaPosition = drive.getFrontLeftSwerveModuleSubsystem().getDriveSubsystem()
                    .getMotorController().getPosition().in(Degrees) / 360 - frontLeftModuleStartPosition;
            double frontRightModuleDeltaPosition = drive.getFrontRightSwerveModuleSubsystem().getDriveSubsystem()
                    .getMotorController().getPosition().in(Degree) / 360 - frontRightModuleStartPosition;
            double rearLeftModuleDeltaPosition = drive.getRearLeftSwerveModuleSubsystem().getDriveSubsystem()
                    .getMotorController().getPosition().in(Degree) / 360 - rearLeftModuleStartPosition;
            double rearRightModuleDeltaPosition = drive.getRearRightSwerveModuleSubsystem().getDriveSubsystem()
                    .getMotorController().getPosition().in(Degree) / 360 - rearRightModuleStartPosition;

            double averageDeltaPosition = (frontLeftModuleDeltaPosition + frontRightModuleDeltaPosition
                    + rearLeftModuleDeltaPosition + rearRightModuleDeltaPosition) / 4;
            // log delta positions and average delta position
            aKitLog.record("frontLeftModuleDeltaPosition", frontLeftModuleDeltaPosition);
            aKitLog.record("frontRightModuleDeltaPosition", frontRightModuleDeltaPosition);
            aKitLog.record("rearLeftModuleDeltaPosition", rearLeftModuleDeltaPosition);
            aKitLog.record("rearRightModuleDeltaPosition", rearRightModuleDeltaPosition);
            aKitLog.record("averageDeltaPosition", averageDeltaPosition);
        }
    }

    @Override
    public void end(boolean interrupted) {
        this.timer.stop();
        drive.stop();
    }
}
