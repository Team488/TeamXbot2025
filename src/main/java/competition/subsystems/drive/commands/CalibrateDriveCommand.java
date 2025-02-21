package competition.subsystems.drive.commands;

import competition.subsystems.drive.DriveSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import xbot.common.command.BaseCommand;
import xbot.common.subsystems.drive.swerve.SwerveModuleSubsystem;

import javax.inject.Inject;

import static edu.wpi.first.units.Units.Degrees;

public class CalibrateDriveCommand extends BaseCommand {
    DriveSubsystem drive;
    Timer timer = new Timer();
    double power = 0.1;
    double frontLeftModuleStartPosition;
    double frontRightModuleStartPosition;
    double rearLeftModuleStartPosition;
    double rearRightModuleStartPosition;
    boolean startingPositionsLogged = false;

    @Inject
    public CalibrateDriveCommand(DriveSubsystem drive) {
        this.drive = drive;
        this.addRequirements(drive);
    }

    @Override
    public void initialize() {
        log.info("Initializing");
        this.timer.restart();
        drive.setAllSwerveModulesToTargetState(new SwerveModuleState(0,new Rotation2d(0)));
    }

    @Override
    public void execute() {
        // wait for modules to be zeroed
        if (timer.hasElapsed(2) && !startingPositionsLogged) {
            // log starting drive motor positions
            frontLeftModuleStartPosition = getDriveMotorPosition(drive.getFrontLeftSwerveModuleSubsystem());
            frontRightModuleStartPosition = getDriveMotorPosition(drive.getFrontRightSwerveModuleSubsystem());
            rearLeftModuleStartPosition = getDriveMotorPosition(drive.getRearLeftSwerveModuleSubsystem());
            rearRightModuleStartPosition = getDriveMotorPosition(drive.getRearRightSwerveModuleSubsystem());

            aKitLog.record("frontLeftModuleStartPosition", frontLeftModuleStartPosition);
            aKitLog.record("frontRightModuleStartPosition", frontRightModuleStartPosition);
            aKitLog.record("rearLeftModuleStartPosition", rearLeftModuleStartPosition);
            aKitLog.record("rearRightModuleStartPosition", rearRightModuleStartPosition);
            startingPositionsLogged = true;
        }
        // after starting positions are logged, start logging the delta
        else if (timer.hasElapsed(2) && startingPositionsLogged) {
            drive.setAllSwerveModulesToTargetState(new SwerveModuleState(power, new Rotation2d(0)));

            double frontLeftModuleDeltaPosition = getDeltaPosition(drive.getFrontLeftSwerveModuleSubsystem(),
                    frontLeftModuleStartPosition);
            double frontRightModuleDeltaPosition = getDeltaPosition(drive.getFrontRightSwerveModuleSubsystem(),
                    frontRightModuleStartPosition);
            double rearLeftModuleDeltaPosition = getDeltaPosition(drive.getRearLeftSwerveModuleSubsystem(),
                    rearLeftModuleStartPosition);
            double rearRightModuleDeltaPosition = getDeltaPosition(drive.getRearRightSwerveModuleSubsystem(),
                    rearRightModuleStartPosition);

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

    private double getDriveMotorPosition(SwerveModuleSubsystem module) {
        return module.getDriveSubsystem().getMotorController()
                .map(mc -> mc.getPosition().in(Degrees) / 360)
                .orElse(0.0);
    }

    private double getDeltaPosition(SwerveModuleSubsystem module, double startingPosition) {
        return getDriveMotorPosition(module) - startingPosition;
    }
}
