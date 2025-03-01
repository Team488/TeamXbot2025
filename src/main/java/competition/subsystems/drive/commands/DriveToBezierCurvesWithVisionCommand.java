package competition.subsystems.drive.commands;

import competition.electrical_contract.ElectricalContract;
import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.pose.PoseSubsystem;
import competition.subsystems.vision.CoprocessorCommunicationSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import static edu.wpi.first.units.Units.Meters;
import org.kobe.xbot.Utilities.Entities.XTableValues;
import xbot.common.logging.RobotAssertionManager;
import xbot.common.properties.PropertyFactory;
import xbot.common.subsystems.drive.SwerveSimpleBezierCommand;
import xbot.common.subsystems.drive.control_logic.HeadingModule;
import xbot.common.trajectory.XbotSwervePoint;

import javax.inject.Inject;
import java.util.AbstractMap;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.concurrent.TimeUnit;
import java.util.stream.Collectors;
import java.util.stream.IntStream;

public class DriveToBezierCurvesWithVisionCommand extends SwerveSimpleBezierCommand {
    private static final int STEPS_PER_SEGMENT = 5;

    final DriveSubsystem drive;
    final PoseSubsystem pose;
    final CoprocessorCommunicationSubsystem coprocessorComms;
    final Distance radiusOfRobot;
    XTableValues.BezierCurves lastBezierCurves;
    Pose2d lastTargetPose;

    @Inject
    DriveToBezierCurvesWithVisionCommand(PoseSubsystem pose, DriveSubsystem drive,
            CoprocessorCommunicationSubsystem coprocessorComms,
            PropertyFactory pf, HeadingModule.HeadingModuleFactory headingModuleFactory,
            RobotAssertionManager assertionManager, ElectricalContract electricalContract) {
        super(drive, pose, pf, headingModuleFactory, assertionManager);
        this.pose = pose;
        this.drive = drive;
        this.coprocessorComms = coprocessorComms;
        this.radiusOfRobot = electricalContract.getRadiusOfRobot();
    }

    @Override
    public void initialize() {
        if (retrieveCurvesFromVision()) {
            super.initialize();
        }
    }

    // allows for driving not in a straight line
    protected void prepareToDriveWithCurves(XTableValues.BezierCurves curves) {
        this.logic.setKeyPoints(this.convertCurvesToSwervePoints(curves));
        this.logic.setConstantVelocity(this.drive.getDriveToWaypointsSpeed().get());
        this.logic.setPrioritizeRotationIfCloseToGoal(true);
        this.logic.setDistanceThresholdToPrioritizeRotation(1.5);
    }

    private List<XbotSwervePoint> convertCurvesToSwervePoints(XTableValues.BezierCurves curves) {
        List<XbotSwervePoint> fullTrajectory = new ArrayList<>();
        var currentStartPoint = this.pose.getCurrentPose2d().getTranslation();

        // Total number of segments and steps.
        var controlPointLists = curves.getCurvesList().stream()
                .map(segment -> segment.getControlPointsList().stream()
                        .map(cp -> new Translation2d(cp.getX(), cp.getY())).collect(Collectors.toList()))
                .filter(cpList -> !cpList.isEmpty()).collect(Collectors.toList());
        int totalSegments = controlPointLists.size();
        int totalSteps = totalSegments * STEPS_PER_SEGMENT;
        int globalStep = 0;

        // Rotation start threshold (10% of the path)
        double rotationStartThreshold = 0.2;

        // Process each Bézier segment.
        for (List<Translation2d> segmentControlPoints : controlPointLists) {
            // The last control point is assumed to be the segment's endpoint.
            Translation2d segmentEndPoint = segmentControlPoints.get(segmentControlPoints.size() - 1);
            List<Translation2d> internalControlPoints = segmentControlPoints.stream().skip(1)
                    .collect(Collectors.toList());

            // Build the full list of points for de Casteljau's algorithm.
            List<Translation2d> allPoints = new ArrayList<>();
            allPoints.add(currentStartPoint);
            allPoints.addAll(internalControlPoints);
            allPoints.add(segmentEndPoint);

            // Process each step in the current segment.
            for (int i = 1; i <= STEPS_PER_SEGMENT; i++) {
                globalStep++;
                double lerpFraction = i / (double) STEPS_PER_SEGMENT;
                Translation2d pointTranslation = deCasteljauIterative(allPoints, lerpFraction);

                // Compute global progress (0 to 1) along the entire trajectory.
                double globalProgress = globalStep / (double) totalSteps;

                Rotation2d targetRotation;

                if (globalProgress < rotationStartThreshold) {
                    // Before 10% progress, maintain the current heading.
                    targetRotation = Rotation2d.fromDegrees(pose.getCurrentHeading().getDegrees());
                } else {
                    // After 10% progress, switch to the target rotation.
                    targetRotation = this.lastTargetPose.getRotation();
                }

                fullTrajectory.add(new XbotSwervePoint(pointTranslation, targetRotation, 1000));
            }

            // Update the starting point for the next segment.
            currentStartPoint = segmentEndPoint;
        }
        return fullTrajectory;
    }

    /**
     * Iterative implementation of de Casteljau's algorithm to compute a point on a
     * Bézier curve.
     *
     * @param points       The control points defining the curve.
     * @param lerpFraction The parametric value (0 to 1) along the curve.
     * @return The computed position as a Translation2d.
     */
    private Translation2d deCasteljauIterative(List<Translation2d> points, double lerpFraction) {
        int n = points.size();
        List<Translation2d> temp = new ArrayList<>(points);

        for (int level = 1; level < n; level++) {
            for (int i = 0; i < n - level; i++) {
                double x = (1 - lerpFraction) * temp.get(i).getX() + lerpFraction * temp.get(i + 1).getX();
                double y = (1 - lerpFraction) * temp.get(i).getY() + lerpFraction * temp.get(i + 1).getY();
                temp.set(i, new Translation2d(x, y));
            }
        }
        return temp.get(0);
    }

    private boolean setTargetPoseForVision(Pose2d targetPose) {
        this.lastTargetPose = targetPose;
        this.log.info("Setting vision target pose: {}", targetPose);
        this.aKitLog.record("Set vision target pose", targetPose);

        return true;
    }

    public static <T, U> List<Map.Entry<T, U>> zipList(List<T> list1, List<U> list2) {
        return IntStream.range(0, Math.min(list1.size(), list2.size()))
                .mapToObj(i -> new AbstractMap.SimpleEntry<T, U>(list1.get(i), list2.get(i)))
                .collect(Collectors.toList());
    }

    public static <T> boolean areListsEqual(List<T> list1, List<T> list2) {
        if (list1.size() != list2.size()) {
            return false;
        }

        var zippedList = zipList(list1, list2);

        for (var zipped : zippedList) {
            if (zipped.getKey() != zipped.getValue()) {
                return false;
            }
        }

        return true;
    }

    // allows for driving not in a straight line
    protected boolean retrieveCurvesFromVision() {
        var currentPose = this.pose.getCurrentPose2d();
        var start = XTableValues.ControlPoint.newBuilder()
                .setY(currentPose.getY())
                .setX(currentPose.getX())
                .setRotationDegrees(currentPose.getRotation().getDegrees())
                .build();

        var end = XTableValues.ControlPoint.newBuilder()
                .setY(this.lastTargetPose.getY())
                .setX(this.lastTargetPose.getX())
                .setRotationDegrees(this.lastTargetPose.getRotation().getDegrees())
                .build();

        var options = XTableValues.TraversalOptions.newBuilder()
                .setMetersPerSecond(this.drive.getMaxTargetSpeedMetersPerSecond())
                .setAccelerationMetersPerSecond(this.drive.getMaxAccelerationMetersPerSecondSquared())
                .setFinalRotationDegrees(this.lastTargetPose.getRotation().getDegrees())
                .setFaceNearestReefAprilTag(true)
                .setSnapToNearestAprilTag(false)
                .setStartFaceNearestReefAprilTagPathThresholdPercentage(0)
                .setEndFaceNearestReefAprilTagPathThresholdPercentage(85)
                .setFaceNearestReefAprilTagDirection(XTableValues.RobotDirection.FRONT)
                .setAprilTagRotationDegreesTurnSpeedFactorPerStep(50)
                .setFinalRotationTurnSpeedFactor(2)
                .build();

        var commander = this.coprocessorComms.getVisionCoprocessorCommander();

        XTableValues.BezierCurves curves = commander
                .requestBezierPathWithOptions(XTableValues.RequestVisionCoprocessorMessage.newBuilder()
                        .setStart(start)
                        .setEnd(end)
                        .setSafeDistanceInches(2) // Will stay an EXTRA 3 inches away (recommended)
                        .setOptions(options)
                        .build(), 5, TimeUnit.SECONDS);
        this.lastBezierCurves = curves;
        this.prepareToDriveWithCurves(this.lastBezierCurves);

        return true;
    }

    protected boolean setDestinationPoseForVision(Pose2d destinationPose, boolean isRearFacing) {
        return this.setDestinationPoseForVision(destinationPose, isRearFacing, Meters.of(0));
    }

    protected boolean setDestinationPoseForVision(Pose2d destinationPose, boolean isRearFacing, Distance additionalDistance) {
        var offsetRotation = Rotation2d.fromDegrees(isRearFacing ? 0 : 180);
        var deltaDistance = this.radiusOfRobot.plus(additionalDistance).in(Meters);
        var deltaTranslation = new Translation2d(deltaDistance, destinationPose.getRotation());
        var destinationTranslation = destinationPose.getTranslation().plus(deltaTranslation);
        var targetPose = new Pose2d(destinationTranslation, destinationPose.getRotation().rotateBy(offsetRotation));

        return this.setTargetPoseForVision(targetPose);
    }

    @Override
    public void execute() {
        super.execute();
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }

}
