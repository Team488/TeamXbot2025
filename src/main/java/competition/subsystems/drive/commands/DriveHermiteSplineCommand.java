package competition.subsystems.drive.commands;

import competition.motion.CubicHermiteSpline;
import competition.motion.CubicHermiteSplineParameters;
import competition.motion.HermiteTrajectory;
import competition.motion.HermiteTrajectoryAdvice;
import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.pose.PoseSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import xbot.common.command.BaseCommand;
import xbot.common.math.XYPair;

import javax.inject.Inject;

public class DriveHermiteSplineCommand extends BaseCommand {

    final DriveSubsystem drive;
    final PoseSubsystem pose;


    HermiteTrajectory trajectory;
    final Translation2d startPoint;

    @Inject
    public DriveHermiteSplineCommand(DriveSubsystem drive, PoseSubsystem pose) {
        this.drive = drive;
        this.pose = pose;

        trajectory = new HermiteTrajectory();
        CubicHermiteSpline spline = new CubicHermiteSpline();
        startPoint = new Translation2d(4.966,5.154);
        spline.setStartPoint(startPoint);
        spline.setEndPoint(new Translation2d(1.2,6.953));
        spline.setStartControlVector(5, Math.toRadians(60));
        spline.setEndControlVector(5, Math.toRadians(160));
        trajectory.setSpline(spline);
    }

    public void setSplineParameters(CubicHermiteSplineParameters parameters) {
        var spline = new CubicHermiteSpline(parameters);
        trajectory.setSpline(spline);
    }

    @Override
    public void initialize() {
        trajectory.initialize(pose.getAbsoluteVelocity(), 0.2);
    }

    @Override
    public void execute() {

        var currentPosition = pose.getCurrentPose2d().getTranslation();
        var advice = trajectory.advise(currentPosition);

        // two components; drive based on pure velocity commands, and PID to ghost.

        // Velocity - spline will be outputting units of meters per second. Need to scale back into % of maximum
        // output.
        XYPair velocityIntent = new XYPair(
                advice.velocity().getX() / drive.getMaxTargetSpeedMetersPerSecond(),
                advice.velocity().getY() / drive.getMaxTargetSpeedMetersPerSecond());
        if (advice.timeFrozen()) {
            velocityIntent = new XYPair(0, 0);
        }

        // Position - spline will be outputting a position that we should be at. We need to PID to that position.
        var positionTranslation = drive.getPowerToAchieveFieldPosition(currentPosition, advice.position());
        XYPair positionIntent = new XYPair(positionTranslation.getX(), positionTranslation.getY());

        var fusedIntent = velocityIntent.add(positionIntent);

        drive.fieldOrientedDrive(fusedIntent, 0, pose.getCurrentHeading().getDegrees(), true);

        aKitLog.record("HermiteGhost", new Pose2d(advice.position(), new Rotation2d()));
    }
}
