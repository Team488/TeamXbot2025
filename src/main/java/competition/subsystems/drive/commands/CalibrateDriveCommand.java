package competition.subsystems.drive.commands;

import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.pose.PoseSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import xbot.common.command.BaseCommand;
import xbot.common.subsystems.drive.swerve.SwerveModuleSubsystem;

import javax.inject.Inject;

import static edu.wpi.first.units.Units.Rotations;

public class CalibrateDriveCommand extends BaseCommand {
    DriveSubsystem drive;
    PoseSubsystem pose;
    Timer timer = new Timer();
    double speed = 1;
    double frontLeftModuleStartPosition;
    double frontRightModuleStartPosition;
    double rearLeftModuleStartPosition;
    double rearRightModuleStartPosition;
    boolean startingPositionsLogged = false;

    private enum CalibrationMode {
        AlignModulesAndWait,
        DriveForwardForTime,
        DecelerateAndWait,
        FinalAnalysis
    }

    CalibrationMode mode = CalibrationMode.AlignModulesAndWait;

    @Inject
    public CalibrateDriveCommand(DriveSubsystem drive, PoseSubsystem pose) {
        this.drive = drive;
        this.pose = pose;
        this.addRequirements(drive);
    }

    @Override
    public void initialize() {
        log.info("Initializing");
        this.timer.restart();
        pose.setCurrentPoseInMeters(new Pose2d(0, 0, new Rotation2d(0)));
        drive.setAllSwerveModulesToTargetState(new SwerveModuleState(0,new Rotation2d(0)));
        mode = CalibrationMode.AlignModulesAndWait;
    }

    @Override
    public void execute() {
        // wait for modules to be zeroed

        switch (mode) {
            case AlignModulesAndWait:
                drive.setAllSwerveModulesToTargetState(new SwerveModuleState(0,new Rotation2d(0)));

                frontLeftModuleStartPosition = getDriveMotorPosition(drive.getFrontLeftSwerveModuleSubsystem());
                frontRightModuleStartPosition = getDriveMotorPosition(drive.getFrontRightSwerveModuleSubsystem());
                rearLeftModuleStartPosition = getDriveMotorPosition(drive.getRearLeftSwerveModuleSubsystem());
                rearRightModuleStartPosition = getDriveMotorPosition(drive.getRearRightSwerveModuleSubsystem());

                aKitLog.record("frontLeftModuleStartPosition", frontLeftModuleStartPosition);
                aKitLog.record("frontRightModuleStartPosition", frontRightModuleStartPosition);
                aKitLog.record("rearLeftModuleStartPosition", rearLeftModuleStartPosition);
                aKitLog.record("rearRightModuleStartPosition", rearRightModuleStartPosition);

                if (timer.hasElapsed(2)) {
                    mode = CalibrationMode.DriveForwardForTime;
                }
                break;
            case DriveForwardForTime:
                drive.setAllSwerveModulesToTargetState(new SwerveModuleState(speed, new Rotation2d(0)));
                logDeltaPositions();
                if (timer.hasElapsed(10)) {
                    mode = CalibrationMode.DecelerateAndWait;
                }
                break;
            case DecelerateAndWait:
                drive.setAllSwerveModulesToTargetState(new SwerveModuleState(0, new Rotation2d(0)));
                logDeltaPositions();
                if (timer.hasElapsed(15)) {
                    mode = CalibrationMode.FinalAnalysis;
                }
                break;
            case FinalAnalysis:
            default:
                // do nothing.
                break;
        }
    }

    private double logDeltaPositions() {
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

        return averageDeltaPosition;
    }

    @Override
    public boolean isFinished() {
        return mode == CalibrationMode.FinalAnalysis;
    }

    @Override
    public void end(boolean interrupted) {
        // Perform final analysis.
        double averageDeltaPosition = logDeltaPositions();

        // How far did the robot think it went?
        log.info("Robot believes it traveled to: " + pose.getCurrentPose2d());
        log.info("Total magnitude of robot's movement: " + pose.getCurrentPose2d().getTranslation().getNorm());
        log.info("This is with the current MetersPerMotorRotation value of "
                + drive.getFrontLeftSwerveModuleSubsystem().getDriveSubsystem().getMetersPerMotorRotation());
        log.info("Average delta in motor rotations: " + averageDeltaPosition);
        log.info("Please check all DeltaPosition (e.g. frontLeftModuleDeltaPosition) values to ensure they are reasonable.");
    }

    private double getDriveMotorPosition(SwerveModuleSubsystem module) {
        return module.getDriveSubsystem().getMotorController()
                .map(mc -> mc.getPosition().in(Rotations))
                .orElse(0.0);
    }

    private double getDeltaPosition(SwerveModuleSubsystem module, double startingPosition) {
        return getDriveMotorPosition(module) - startingPosition;
    }
}
