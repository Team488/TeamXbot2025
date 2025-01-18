package competition.subsystems.drive.commands;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.pose.PoseSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Timer;

import org.littletonrobotics.junction.Logger;
import xbot.common.command.BaseCommand;
import xbot.common.math.PIDManager;
import xbot.common.math.XYPair;
import xbot.common.subsystems.drive.control_logic.HeadingModule;

import javax.inject.Inject;

public class FollowPathCommand extends BaseCommand {
    private final Timer timer = new Timer();
    private final DriveSubsystem drive;
    private PathPlannerPath path;
    private final PoseSubsystem pose;
    private PathPlannerTrajectory trajectory;
    PIDManager translationPID;
    HeadingModule headingModule;
    RobotConfig robotConfig;
    double massKg = 68.039;
    double momentOfInertia = 20.250;
    ModuleConfig moduleConfig;
    double wheelRadius = 0.051;
    double maxDriveVelocityMPS = 3.5;
    double wheelCOF = 1.0;
    double driveGearing = 6.120;
    double driveCurrentLimits = 35.0;
    int numMotors = 8;
    DCMotor dcMotor = DCMotor.getKrakenX60(numMotors);
    double trackWidth = 0.584;

    @Inject
    public FollowPathCommand(DriveSubsystem driveSubsystem, PoseSubsystem pose,
                             HeadingModule.HeadingModuleFactory headingModuleFactory) {
        this.drive = driveSubsystem;
        this.pose = pose;
        this.moduleConfig = new ModuleConfig(wheelRadius, maxDriveVelocityMPS,
                wheelCOF, dcMotor, driveGearing, driveCurrentLimits, numMotors);
        this.robotConfig = new RobotConfig(massKg, momentOfInertia, moduleConfig, trackWidth);

        translationPID = drive.getPathPlannerTranslationPid();
        headingModule = headingModuleFactory.create(drive.getPathPlannerRotationPid());

        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        log.info("Initializing");
        translationPID.reset();
        headingModule.reset();

        trajectory = new PathPlannerTrajectory(
                path,
                drive.getSwerveDriveKinematics().toChassisSpeeds(drive.getSwerveModuleStates()),
                pose.getCurrentPose2d().getRotation(), robotConfig);

        this.timer.restart();
    }

    @Override
    public void execute() {
        double currentTime = this.timer.get();
        PathPlannerTrajectoryState desiredState =  trajectory.sample(currentTime);
        Pose2d currentPose = pose.getCurrentPose2d();

        // Converting the velocity from a vector to x and y
        double vx = desiredState.linearVelocity * Math.cos(desiredState.heading.getRadians());
        double vy = desiredState.linearVelocity * Math.sin(desiredState.heading.getRadians());

        // PID to keep robot on the path
        double vxFeedBack = translationPID.calculate
                (currentPose.getX(), desiredState.pose.getX());
        double vyFeedBack = translationPID.calculate(
                currentPose.getY(), desiredState.pose.getY());

        // Calculate omega
        double omega = headingModule.calculateHeadingPower(desiredState.heading.getRadians());

        // Convert Field relative chassis speeds to robot relative
        ChassisSpeeds chassisSpeeds =
                ChassisSpeeds.fromFieldRelativeSpeeds(vx + vxFeedBack, vy + vyFeedBack,
                        omega, currentPose.getRotation());

        driveRobotRelative(chassisSpeeds);

        Logger.recordOutput("PathPlanner/FollowPathCommand/DesiredStatePose", desiredState.pose);
        Logger.recordOutput("PathPlanner/FollowPathCommand/desiredVXPerSecond", vx);
        Logger.recordOutput("PathPlanner/FollowPathCommand/desiredVYPerSecond", vy);
        Logger.recordOutput("PathPlanner/FollowPathCommand/vxFeedBack", vxFeedBack);
        Logger.recordOutput("PathPlanner/FollowPathCommand/vyFeedback", vyFeedBack);
        Logger.recordOutput("PathPlanner/FollowPathCommand/desiredOmega", omega);
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(trajectory.getTotalTimeSeconds());
    }

    @Override
    public void end(boolean interrupted) {
        log.info("Command has ended");
        this.timer.stop(); // Stop timer
        driveRobotRelative(new ChassisSpeeds(0, 0, 0));
        drive.stop();
    }



    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
        //AdvantageScope Logging
        double vX = robotRelativeSpeeds.vxMetersPerSecond;
        double vY = robotRelativeSpeeds.vyMetersPerSecond;
        double omegaRad = robotRelativeSpeeds.omegaRadiansPerSecond;

        //getting chassis speeds and dividing it by top speeds, so it can be inputted into move()
        double calcX = robotRelativeSpeeds.vxMetersPerSecond / drive.getMaxTargetSpeedMetersPerSecond();
        double calcY = robotRelativeSpeeds.vyMetersPerSecond / drive.getMaxTargetSpeedMetersPerSecond();
        double calcAng = robotRelativeSpeeds.omegaRadiansPerSecond / drive.getMaxTargetTurnRate();

        XYPair xySpeeds = new XYPair(calcX, calcY);

        drive.move(xySpeeds, calcAng, pose.getCurrentPose2d());

        Logger.recordOutput("PathPlanner/FollowPathCommand/vxMetersPerSecond", vX);
        Logger.recordOutput("PathPlanner/FollowPathCommand/vyMetersPerSecond", vY);
        Logger.recordOutput("PathPlanner/FollowPathCommand/omegaRadPerSecond", omegaRad);
    }

    public void setPath(PathPlannerPath path) {
        this.path = path;
    }

    public Pose2d getStartingPose() {
        return trajectory.getInitialState().pose;
    }
}