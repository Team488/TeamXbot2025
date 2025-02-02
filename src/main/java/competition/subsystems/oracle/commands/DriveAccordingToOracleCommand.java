package competition.subsystems.oracle.commands;

import competition.subsystems.oracle.OracleDriveAdvice;
import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.oracle.OracleSubsystem;
import competition.subsystems.pose.PoseSubsystem;
import edu.wpi.first.math.geometry.Twist2d;
import xbot.common.command.BaseCommand;
import xbot.common.logging.RobotAssertionManager;
import xbot.common.math.XYPair;
import xbot.common.properties.PropertyFactory;
import xbot.common.subsystems.drive.SwervePointKinematics;
import xbot.common.subsystems.drive.SwerveSimpleTrajectoryMode;
import xbot.common.subsystems.drive.control_logic.HeadingModule;
import xbot.common.trajectory.SwerveSimpleTrajectoryLogic;

import javax.inject.Inject;

public class DriveAccordingToOracleCommand extends BaseCommand {

    final DriveSubsystem drive;
    final PoseSubsystem pose;
    final HeadingModule headingModule;
    public final SwerveSimpleTrajectoryLogic logic;
    OracleSubsystem oracle;
    private int lastSeenInstructionNumber = -1;

    @Inject
    public DriveAccordingToOracleCommand(DriveSubsystem drive, PoseSubsystem pose,
                                         PropertyFactory pf, HeadingModule.HeadingModuleFactory headingModuleFactory,
                                         OracleSubsystem oracle, RobotAssertionManager assertionManager) {
        this.drive = drive;
        this.pose = pose;
        headingModule = headingModuleFactory.create(drive.getRotateToHeadingPid());
        this.oracle = oracle;

        pf.setPrefix(this);
        this.addRequirements(drive);
        logic = new SwerveSimpleTrajectoryLogic(assertionManager);

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        log.info("Initializing");
        oracle.requestReevaluation();
        setNewInstruction();
    }

    private void setNewInstruction() {
        OracleDriveAdvice advice = oracle.getDriveAdvice();
        lastSeenInstructionNumber = advice.instructionNumber;

        logic.setVelocityMode(SwerveSimpleTrajectoryMode.GlobalKinematicsValue);
        logic.setGlobalKinematicValues(new SwervePointKinematics(2, 0, pose.getAbsoluteVelocity(), 4));
        logic.setKeyPoints(advice.path);

        logic.reset(pose.getCurrentPose2d());
    }

    @Override
    public void execute() {
        if (oracle.getDriveAdvice().instructionNumber != lastSeenInstructionNumber) {
            setNewInstruction();
        }

        Twist2d powers = logic.calculatePowers(pose.getCurrentPose2d(), drive.getPositionalPid(), headingModule, drive.getMaxTargetSpeedMetersPerSecond());

        aKitLog.record("Powers", powers);

        drive.fieldOrientedDrive(
                new XYPair(powers.dx, powers.dy),
                powers.dtheta, pose.getCurrentHeading().getDegrees(), false);
    }
}
