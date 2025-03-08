package competition.subsystems.drive.commands;

import competition.motion.CommonRouteGenerator;
import competition.motion.CubicHermiteSpline;
import competition.motion.CubicHermiteSplineParameters;
import competition.motion.HermiteTrajectory;
import competition.motion.HermiteTrajectoryAdvice;
import competition.operator_interface.OperatorCommandMap;
import competition.operator_interface.OperatorInterface;
import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.pose.Landmarks;
import competition.subsystems.pose.PoseSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import xbot.common.command.BaseCommand;
import xbot.common.math.XYPair;

import javax.inject.Inject;
import java.sql.Array;
import java.util.ArrayList;
import java.util.Collections;

public class DriveHermiteSplineCommand extends BaseCommand {

    final DriveSubsystem drive;
    final PoseSubsystem pose;
    final OperatorInterface oi;
    final CommonRouteGenerator routeGenerator;

    HermiteTrajectory trajectory;
    final Translation2d startPoint;

    @Inject
    public DriveHermiteSplineCommand(DriveSubsystem drive, PoseSubsystem pose, OperatorInterface oi,
                                     CommonRouteGenerator routeGenerator) {
        this.drive = drive;
        this.pose = pose;
        this.oi = oi;
        this.routeGenerator = routeGenerator;

        trajectory = new HermiteTrajectory();
        CubicHermiteSpline spline = new CubicHermiteSpline();
        startPoint = new Translation2d(4.966,5.154);
        spline.setStartPoint(startPoint);
        spline.setEndPoint(new Translation2d(1.2,6.953));
        spline.setStartControlVector(5, Math.toRadians(60));
        spline.setEndControlVector(5, Math.toRadians(160));
        trajectory.setSpline(spline);

        var splineAParams = new CubicHermiteSplineParameters(
                new Translation2d(1, 1),
                new Translation2d(5, 1),
                new Translation2d(5, Rotation2d.fromDegrees(0)),
                new Translation2d(5, Rotation2d.fromDegrees(0))
        );
        var splineBParams = new CubicHermiteSplineParameters(
                new Translation2d(5, 1),
                new Translation2d(7, 7),
                new Translation2d(5, Rotation2d.fromDegrees(0)),
                new Translation2d(5, Rotation2d.fromDegrees(90))
        );

        var splineA = new CubicHermiteSpline(splineAParams);
        var splineB = new CubicHermiteSpline(splineBParams);
        var splineList = new ArrayList<CubicHermiteSpline>();
        splineList.add(splineA);
        splineList.add(splineB);

        addRequirements(drive);
    }

    @Override
    public void initialize() {

        var closestFace = pose.getClosestReefFace();

        var route = routeGenerator.getRouteFromReefToLoadingStation(closestFace, Landmarks.CoralStation.RIGHT);
        trajectory.setSplines(route);

        trajectory.initialize(pose.getAbsoluteVelocity(), 1);
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

        aKitLog.record("HermiteVelocityRaw", velocityIntent.getMagnitude());

        if (advice.timeFrozen()) {
            // If the timer is frozen, that means we have gone off-track somewhere and need to catch back up. We have to
            // be careful here - if we just set the velocity component to 0, we will use PID to approach, but as soon as we
            // do the ghost will shoot ahead as it expects us to have much higher velocities. Instead, we need to use
            // the portion of the velocity that is in the direction of the ghost. We can find the similarity of the vector
            // using the dot product, and use that dot product to scale the velocity vector.

            var directionToGhost = advice.position().minus(currentPosition);
            var directionToGhostUnitVector = XYPair.fromUnitPolar(directionToGhost.getAngle().getDegrees());
            var velocityIntentUnitVector = XYPair.fromUnitPolar(velocityIntent.getAngle());

            var similarity = directionToGhostUnitVector.dotProduct(velocityIntentUnitVector);
            velocityIntent = velocityIntent.clone().scale(similarity);
        }

        aKitLog.record("HermiteVelocityAdjusted", velocityIntent.getMagnitude());

        // Position - spline will be outputting a position that we should be at. We need to PID to that position.
        var positionTranslation = drive.getPowerToAchieveFieldPosition(currentPosition, advice.position());
        XYPair positionIntent = new XYPair(positionTranslation.getX(), positionTranslation.getY());

        var fusedIntent = velocityIntent.add(positionIntent);

        if (oi.driverGamepad.getLeftVector().getNorm() > 0.2) {
            fusedIntent = new XYPair(oi.driverGamepad.getLeftVector().getX(), oi.driverGamepad.getLeftVector().getY());
        }

        drive.fieldOrientedDrive(fusedIntent, 0, pose.getCurrentHeading().getDegrees(), true);

        aKitLog.record("HermiteGhost", new Pose2d(advice.position(), new Rotation2d()));

        timeLeft = advice.timeRemaining();
        aKitLog.record("TimeLeft", timeLeft);
    }

    double timeLeft;

    @Override
    public boolean isFinished() {
        return timeLeft < 0.05;
    }
}
