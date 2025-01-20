package competition.subsystems.drive.commands;

import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.pose.PoseSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;

import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.Logger;
import xbot.common.command.BaseCommand;
import xbot.common.math.XYPair;
import xbot.common.subsystems.drive.control_logic.HeadingModule;

import javax.inject.Inject;
import java.io.IOException;

public class FollowPathCommand extends BaseCommand {
    private final Timer timer = new Timer();
    private final DriveSubsystem drive;
    private PathPlannerPath path;
    private final PoseSubsystem pose;
    private PathPlannerTrajectory trajectory;
    HeadingModule headingModule;
    PIDController translationPID;
    RobotConfig robotConfig;


    @Inject
    public FollowPathCommand(DriveSubsystem driveSubsystem, PoseSubsystem pose,
                             HeadingModule.HeadingModuleFactory headingModuleFactory) {
        this.drive = driveSubsystem;
        this.pose = pose;
        headingModule = headingModuleFactory.create(drive.getPathPlannerRotationPid());
        translationPID = new PIDController(5,0,0);

        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        log.info("Initializing");
        translationPID.reset();
        headingModule.reset();

        try {
            robotConfig = RobotConfig.fromGUISettings();
        } catch (ParseException | IOException e) {
            log.info("RobotConfig problem");
            cancel();
        }

        trajectory = new PathPlannerTrajectory(
                path,
                drive.getSwerveDriveKinematics().toChassisSpeeds(drive.getSwerveModuleStates()),
                pose.getCurrentPose2d().getRotation(), robotConfig);

        this.timer.restart();
        Logger.recordOutput("PathPlanner/FollowPathCommand/trajectoryTime", trajectory.getTotalTimeSeconds());
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
        double omega = headingModule.calculateHeadingPower(desiredState.pose.getRotation().getDegrees());

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
        Logger.recordOutput("PathPlanner/FollowPathCommand/timer", timer.get());
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
    public Pose2d getPathStartingPose() {
        return path.getStartingHolonomicPose().get();
    }
}