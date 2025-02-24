package competition.subsystems.drive.commands;

import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.pose.PoseSubsystem;
import competition.subsystems.vision.CoprocessorCommunicationSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.kobe.xbot.JClient.XTablesClient;
import org.kobe.xbot.JClient.XTablesClientManager;
import org.kobe.xbot.Utilities.VisionCoprocessorCommander;
import org.kobe.xbot.Utilities.Entities.XTableValues;
import org.kobe.xbot.Utilities.Entities.VisionCoprocessor;
import xbot.common.logging.RobotAssertionManager;
import xbot.common.properties.PropertyFactory;
import xbot.common.subsystems.drive.SwerveSimpleBezierCommand;
import xbot.common.subsystems.drive.control_logic.HeadingModule;
import xbot.common.trajectory.XbotSwervePoint;

import javax.inject.Inject;
import java.util.AbstractMap;
import java.util.Arrays;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.concurrent.TimeUnit;
import java.util.stream.Collectors;
import java.util.stream.IntStream;

public class DriveToBezierCurvesWithVisionCommand extends SwerveSimpleBezierCommand {
    DriveSubsystem drive;
    PoseSubsystem pose;
    CoprocessorCommunicationSubsystem coprocessorComms;
    XTableValues.BezierCurves lastBezierCurves;
    Pose2d lastTargetPose;

    @Inject
    DriveToBezierCurvesWithVisionCommand(PoseSubsystem pose, DriveSubsystem drive, CoprocessorCommunicationSubsystem coprocessorComms,
                                         PropertyFactory pf, HeadingModule.HeadingModuleFactory headingModuleFactory,
                                         RobotAssertionManager assertionManager) {
        super(drive, pose, pf, headingModuleFactory, assertionManager);
        this.pose = pose;
        this.drive = drive;
        this.coprocessorComms = coprocessorComms;
    }

    @Override
    public void initialize() {
        if (retrieveCurvesFromVision()) {
            super.initialize();
        }
    }

    //allows for driving not in a straight line
    public void prepareToDriveWithCurves(XTableValues.BezierCurves curves) {
        List<XbotSwervePoint> swervePoints = new ArrayList<>();
        
        for (XTableValues.BezierCurve curve : curves.getCurvesList()) {
            for (XTableValues.ControlPoint controlPoint : curve.getControlPointsList()) {
                swervePoints.add(new XbotSwervePoint(controlPoint.getX(), controlPoint.getY(), controlPoint.getRotationDegrees(),
                                                     this.drive.getDriveToWaypointsDurationPerPoint().get()));

                this.log.info("Adding swerve point: {}", controlPoint.getRotationDegrees());
            }
        }

        this.logic.setKeyPoints(swervePoints);
        this.logic.setConstantVelocity(this.drive.getDriveToWaypointsSpeed().get());
    }

    protected boolean setTargetPoseForVision(Pose2d targetPose) {
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

    //allows for driving not in a straight line
    public boolean retrieveCurvesFromVision() {
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
        
        try (VisionCoprocessorCommander commander = new VisionCoprocessorCommander(VisionCoprocessor.LOCALHOST)) {

            XTableValues.BezierCurves curves = commander
                    .requestBezierPathWithOptions(XTableValues.RequestVisionCoprocessorMessage.newBuilder()
                            .setStart(start)
                            .setEnd(end)
                            .setSafeDistanceInches(3) // Will stay an EXTRA 3 inches away (recommended)
                            .setOptions(options)
                            .build(), 5, TimeUnit.SECONDS);
            this.lastBezierCurves = curves;
            this.prepareToDriveWithCurves(this.lastBezierCurves);
        }

        return true;
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
