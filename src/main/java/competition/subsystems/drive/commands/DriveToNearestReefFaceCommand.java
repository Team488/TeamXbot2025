package competition.subsystems.drive.commands;

import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.pose.Landmarks;
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

    Pose2d BlueCloseReefFace = Landmarks.BlueCloseAlgae;
    Pose2d BlueCloseRightReefFace = Landmarks.BlueCloseRightAlgae;
    Pose2d BlueCloseLeftReefFace = Landmarks.BlueCloseLeftAlgae;
    Pose2d BlueFarLeftReefFace = Landmarks.BlueFarLeftAlgae;
    Pose2d BlueFarReefFace = Landmarks.BlueFarAlgae;
    Pose2d BlueFarRightReefFace = Landmarks.BlueFarRightAlgae;

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
        double farLeftDistance = PoseSubsystem.convertBlueToRedIfNeeded(
                BlueFarLeftReefFace).getTranslation().getDistance(currentPose.getTranslation());
        double farDistance = PoseSubsystem.convertBlueToRedIfNeeded(
                BlueFarReefFace).getTranslation().getDistance(currentPose.getTranslation());
        double farRightDistance = PoseSubsystem.convertBlueToRedIfNeeded(
                BlueFarRightReefFace).getTranslation().getDistance(currentPose.getTranslation());

        HashMap<Double, Pose2d> hashMap = new HashMap<>();
        hashMap.put(closeLeftDistance, PoseSubsystem.convertBlueToRedIfNeeded(BlueCloseLeftReefFace));
        hashMap.put(closeDistance, PoseSubsystem.convertBlueToRedIfNeeded(BlueCloseReefFace));
        hashMap.put(closeRightDistance, PoseSubsystem.convertBlueToRedIfNeeded(BlueCloseRightReefFace));
        hashMap.put(farLeftDistance, PoseSubsystem.convertBlueToRedIfNeeded(BlueFarLeftReefFace));
        hashMap.put(farDistance, PoseSubsystem.convertBlueToRedIfNeeded(BlueFarReefFace));
        hashMap.put(farRightDistance, PoseSubsystem.convertBlueToRedIfNeeded(BlueFarRightReefFace));

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
            hashMap.put(PoseSubsystem.convertBlueToRedIfNeeded(BlueFarLeftReefFace), 9);
            hashMap.put(PoseSubsystem.convertBlueToRedIfNeeded(BlueFarReefFace), 10);
            hashMap.put(PoseSubsystem.convertBlueToRedIfNeeded(BlueFarRightReefFace), 11);
        }
        else {
            hashMap.put(BlueCloseLeftReefFace, 19);
            hashMap.put(BlueCloseReefFace, 18);
            hashMap.put(BlueCloseRightReefFace, 17);
            hashMap.put(BlueFarLeftReefFace, 20);
            hashMap.put(BlueFarReefFace, 21);
            hashMap.put(BlueFarRightReefFace, 22);
        }
        aKitLog.record("TargetAprilTagID", hashMap.get(targetReefFacePose));

        return hashMap.get(targetReefFacePose);
    }

}

