package competition.subsystems.drive.commands;

import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.pose.PoseSubsystem;
import competition.subsystems.vision.AprilTagVisionSubsystemExtended;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import xbot.common.properties.PropertyFactory;
import xbot.common.subsystems.drive.SwerveSimpleTrajectoryCommand;
import xbot.common.subsystems.drive.control_logic.HeadingModule;
import xbot.common.trajectory.XbotSwervePoint;

import javax.inject.Inject;
import java.util.ArrayList;
import java.util.HashMap;


public class DriveToNearestReefFaceCommand extends SwerveSimpleTrajectoryCommand {

    public static Pose2d BlueCloseReefFace = new Pose2d(3.158, 4.026, Rotation2d.fromDegrees(0));
    public static Pose2d BlueCloseRightReefFace = new Pose2d(3.829, 2.880, Rotation2d.fromDegrees(60));
    public static Pose2d BlueCloseLeftReefFace = new Pose2d(3.829, 5.180, Rotation2d.fromDegrees(-60));
    public static Pose2d BlueBackLeftReefFace = new Pose2d(5.150, 5.180, Rotation2d.fromDegrees(-120));
    public static Pose2d BlueBackReefFace = new Pose2d(5.850, 4.026 , Rotation2d.fromDegrees(-180));
    public static Pose2d BlueBackRightReefFace = new Pose2d(5.150, 2.880, Rotation2d.fromDegrees(120));

    DriveSubsystem drive;
    AprilTagVisionSubsystemExtended aprilTagVisionSubsystem;
    Pose2d targetReefFacePose;

    @Inject
    public DriveToNearestReefFaceCommand(DriveSubsystem drive, PoseSubsystem pose, PropertyFactory pf,
                                         HeadingModule.HeadingModuleFactory headingModuleFactory,
                                         AprilTagVisionSubsystemExtended aprilTagVisionSubsystem) {
        super(drive, pose, pf, headingModuleFactory);
        this.drive = drive;
        this.aprilTagVisionSubsystem = aprilTagVisionSubsystem;
    }

    @Override
    public void initialize() {
        log.info("Initializing");

        targetReefFacePose = getClosestReefFacePose();

        ArrayList<XbotSwervePoint> swervePoints = new ArrayList<>();
        swervePoints.add(new XbotSwervePoint(targetReefFacePose, 10));
        this.logic.setKeyPoints(swervePoints);
        this.logic.setEnableConstantVelocity(true);
        this.logic.setConstantVelocity(drive.getMaxTargetSpeedMetersPerSecond());
        super.initialize();
    }

    @Override
    public void execute() {
        super.execute();
        aKitLog.record("targetReefFacePose", targetReefFacePose);
    }

    @Override
    public boolean isFinished() {
        return aprilTagVisionSubsystem.reefAprilTagCameraHasCorrectTarget(getTargetAprilTagID())
                || logic.recommendIsFinished(pose.getCurrentPose2d(), drive.getPositionalPid(), headingModule);
    }

    private Pose2d getClosestReefFacePose() {
        Pose2d currentPose = pose.getCurrentPose2d();

        double closeDistance = PoseSubsystem.convertBlueToRedIfNeeded(
                BlueCloseReefFace).getTranslation().getDistance(currentPose.getTranslation());
        double closeLeftDistance = PoseSubsystem.convertBlueToRedIfNeeded(
                BlueCloseLeftReefFace).getTranslation().getDistance(currentPose.getTranslation());
        double closeRightDistance = PoseSubsystem.convertBlueToRedIfNeeded(
                BlueCloseRightReefFace).getTranslation().getDistance(currentPose.getTranslation());
        double backLeftDistance = PoseSubsystem.convertBlueToRedIfNeeded(
                BlueBackLeftReefFace).getTranslation().getDistance(currentPose.getTranslation());
        double backDistance = PoseSubsystem.convertBlueToRedIfNeeded(
                BlueBackReefFace).getTranslation().getDistance(currentPose.getTranslation());
        double backRightDistance = PoseSubsystem.convertBlueToRedIfNeeded(
                BlueBackRightReefFace).getTranslation().getDistance(currentPose.getTranslation());

        HashMap<Double, Pose2d> hashMap = new HashMap<>();
        hashMap.put(closeLeftDistance, PoseSubsystem.convertBlueToRedIfNeeded(BlueCloseLeftReefFace));
        hashMap.put(closeDistance, PoseSubsystem.convertBlueToRedIfNeeded(BlueCloseReefFace));
        hashMap.put(closeRightDistance, PoseSubsystem.convertBlueToRedIfNeeded(BlueCloseRightReefFace));
        hashMap.put(backLeftDistance, PoseSubsystem.convertBlueToRedIfNeeded(BlueBackLeftReefFace));
        hashMap.put(backDistance, PoseSubsystem.convertBlueToRedIfNeeded(BlueBackReefFace));
        hashMap.put(backRightDistance, PoseSubsystem.convertBlueToRedIfNeeded(BlueBackRightReefFace));

        double leastDistance = closeLeftDistance;

        for (Double distance : hashMap.keySet()) {
            if (distance < leastDistance) {
                leastDistance = distance;
            }
        }
        return hashMap.get(leastDistance);
    }

    private int getTargetAprilTagID() {
        HashMap<Pose2d, Integer> hashMap = new HashMap<>();

        // Note: flipped april tag IDs across the y-midpoint of the field
        if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red) {
            hashMap.put(PoseSubsystem.convertBlueToRedIfNeeded(BlueCloseLeftReefFace), 8);
            hashMap.put(PoseSubsystem.convertBlueToRedIfNeeded(BlueCloseReefFace), 7);
            hashMap.put(PoseSubsystem.convertBlueToRedIfNeeded(BlueCloseRightReefFace), 6);
            hashMap.put(PoseSubsystem.convertBlueToRedIfNeeded(BlueBackLeftReefFace), 9);
            hashMap.put(PoseSubsystem.convertBlueToRedIfNeeded(BlueBackReefFace), 10);
            hashMap.put(PoseSubsystem.convertBlueToRedIfNeeded(BlueBackRightReefFace), 11);
        }
        else {
            hashMap.put(BlueCloseLeftReefFace, 19);
            hashMap.put(BlueCloseReefFace, 18);
            hashMap.put(BlueCloseRightReefFace, 17);
            hashMap.put(BlueBackLeftReefFace, 20);
            hashMap.put(BlueBackReefFace, 21);
            hashMap.put(BlueBackRightReefFace, 22);
        }
        aKitLog.record("TargetAprilTagID", hashMap.get(targetReefFacePose));

        return hashMap.get(targetReefFacePose);
    }

}

