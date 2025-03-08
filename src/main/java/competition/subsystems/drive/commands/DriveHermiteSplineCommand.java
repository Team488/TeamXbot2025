package competition.subsystems.drive.commands;

import competition.motion.CommonRouteGenerator;
import competition.motion.CubicHermiteSpline;
import competition.motion.HermiteTrajectory;
import competition.operator_interface.OperatorInterface;
import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.pose.Landmarks;
import competition.subsystems.pose.PoseSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import xbot.common.command.BaseCommand;
import xbot.common.math.XYPair;
import xbot.common.subsystems.drive.control_logic.HeadingModule;

import javax.inject.Inject;
import java.util.ArrayList;

public class DriveHermiteSplineCommand extends BaseCommand {

    final DriveSubsystem drive;
    final PoseSubsystem pose;
    final OperatorInterface oi;
    final CommonRouteGenerator routeGenerator;

    HermiteTrajectory trajectory;

    private Landmarks.ReefFace endingReefFace;
    private Landmarks.Branch endingBranch;

    private Landmarks.CoralStation endingCoralStation;
    private double distanceToStartUsingFinalHeading = 2.5;
    final private HeadingModule headingModule;
    Rotation2d initialHeading;
    Rotation2d endingHeading;

    public enum TargetMode {
        Reef,
        Station
    }
    private TargetMode mode;

    @Inject
    public DriveHermiteSplineCommand(DriveSubsystem drive, PoseSubsystem pose, OperatorInterface oi,
                                     CommonRouteGenerator routeGenerator, HeadingModule.HeadingModuleFactory headingModulefactory) {
        this.drive = drive;
        this.pose = pose;
        this.oi = oi;
        this.routeGenerator = routeGenerator;
        trajectory = new HermiteTrajectory();
        mode = TargetMode.Reef;

        this.headingModule = headingModulefactory.create(drive.getRotateToHeadingPid());

        addRequirements(drive);
    }

    public void configureForReef(Landmarks.ReefFace reefFace, Landmarks.Branch branch) {
        mode = TargetMode.Reef;

        endingReefFace = reefFace;
        endingBranch = branch;
    }

    public void configureForStation(Landmarks.CoralStation coralStation) {
        mode = TargetMode.Station;

        endingCoralStation = coralStation;
    }

    private void setDistanceToStartUsingFinalHeading(double seconds) {
        distanceToStartUsingFinalHeading = seconds;
    }

    @Override
    public void initialize() {
        log.info("Initializing");
        // In all cases we need the nearest loading station
        var closestStation = pose.getClosestCoralStation();
        initialHeading = pose.getCurrentHeading();
        var route = new ArrayList<CubicHermiteSpline>();
        boolean flipFinalHeading = false;

        if (mode == TargetMode.Reef) {
            route = routeGenerator.getRouteFromLoadingStationToReef(
                    closestStation,
                    endingReefFace,
                    endingBranch
            );
        } else if (mode == TargetMode.Station) {
            var closestFace = pose.getClosestReefFace();
            route = routeGenerator.getRouteFromReefToLoadingStation(
                    closestFace,
                    endingCoralStation);
            flipFinalHeading = true;
        }

        trajectory.setSplines(route);
        endingHeading = route.get(route.size()-1).getEndControlVector().getAngle();
        if (flipFinalHeading) {
            endingHeading = endingHeading.rotateBy(Rotation2d.fromDegrees(180));
        }

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

        if (fusedIntent.getMagnitude() > 1) {
            fusedIntent = fusedIntent.clone().scale(1 / fusedIntent.getMagnitude());
        }

        double rotateIntent = 0;
        if (advice.timeRemaining() < distanceToStartUsingFinalHeading) {
            rotateIntent = headingModule.calculateHeadingPower(endingHeading);
        }

        drive.fieldOrientedDrive(fusedIntent, rotateIntent, pose.getCurrentHeading().getDegrees(), true);

        aKitLog.record("HermiteGhost", new Pose2d(advice.position(), new Rotation2d()));

        distanceRemaining = advice.timeRemaining();
        aKitLog.record("DistanceRemaining", distanceRemaining);
    }

    double distanceRemaining;

    @Override
    public boolean isFinished() {
        return distanceRemaining < 0.05;
    }
}
