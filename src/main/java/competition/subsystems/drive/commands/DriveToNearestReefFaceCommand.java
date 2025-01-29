package competition.subsystems.drive.commands;

import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.pose.Landmarks;
import competition.subsystems.pose.PoseSubsystem;
import competition.subsystems.vision.AprilTagVisionSubsystemExtended;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import xbot.common.properties.PropertyFactory;
import xbot.common.subsystems.drive.SwerveSimpleTrajectoryCommand;
import xbot.common.subsystems.drive.control_logic.HeadingModule;
import xbot.common.trajectory.XbotSwervePoint;

import javax.inject.Inject;
import java.util.ArrayList;
import java.util.HashMap;


public class DriveToNearestReefFaceCommand extends SwerveSimpleTrajectoryCommand {

    Pose2d blueCloseReefFace = Landmarks.BlueCloseAlgae;
    Pose2d blueCloseRightReefFace = Landmarks.BlueCloseRightAlgae;
    Pose2d blueCloseLeftReefFace = Landmarks.BlueCloseLeftAlgae;
    Pose2d blueFarLeftReefFace = Landmarks.BlueFarLeftAlgae;
    Pose2d blueFarReefFace = Landmarks.BlueFarAlgae;
    Pose2d blueFarRightReefFace = Landmarks.BlueFarRightAlgae;

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
                blueCloseReefFace).getTranslation().getDistance(currentPose.getTranslation());
        double closeLeftDistance = PoseSubsystem.convertBlueToRedIfNeeded(
                blueCloseLeftReefFace).getTranslation().getDistance(currentPose.getTranslation());
        double closeRightDistance = PoseSubsystem.convertBlueToRedIfNeeded(
                blueCloseRightReefFace).getTranslation().getDistance(currentPose.getTranslation());
        double farLeftDistance = PoseSubsystem.convertBlueToRedIfNeeded(
                blueFarLeftReefFace).getTranslation().getDistance(currentPose.getTranslation());
        double farDistance = PoseSubsystem.convertBlueToRedIfNeeded(
                blueFarReefFace).getTranslation().getDistance(currentPose.getTranslation());
        double farRightDistance = PoseSubsystem.convertBlueToRedIfNeeded(
                blueFarRightReefFace).getTranslation().getDistance(currentPose.getTranslation());

        HashMap<Double, Pose2d> hashMap = new HashMap<>();
        hashMap.put(closeLeftDistance, PoseSubsystem.convertBlueToRedIfNeeded(blueCloseLeftReefFace));
        hashMap.put(closeDistance, PoseSubsystem.convertBlueToRedIfNeeded(blueCloseReefFace));
        hashMap.put(closeRightDistance, PoseSubsystem.convertBlueToRedIfNeeded(blueCloseRightReefFace));
        hashMap.put(farLeftDistance, PoseSubsystem.convertBlueToRedIfNeeded(blueFarLeftReefFace));
        hashMap.put(farDistance, PoseSubsystem.convertBlueToRedIfNeeded(blueFarReefFace));
        hashMap.put(farRightDistance, PoseSubsystem.convertBlueToRedIfNeeded(blueFarRightReefFace));

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
            hashMap.put(PoseSubsystem.convertBlueToRedIfNeeded(blueCloseLeftReefFace), 8);
            hashMap.put(PoseSubsystem.convertBlueToRedIfNeeded(blueCloseReefFace), 7);
            hashMap.put(PoseSubsystem.convertBlueToRedIfNeeded(blueCloseRightReefFace), 6);
            hashMap.put(PoseSubsystem.convertBlueToRedIfNeeded(blueFarLeftReefFace), 9);
            hashMap.put(PoseSubsystem.convertBlueToRedIfNeeded(blueFarReefFace), 10);
            hashMap.put(PoseSubsystem.convertBlueToRedIfNeeded(blueFarRightReefFace), 11);
        }
        else {
            hashMap.put(blueCloseLeftReefFace, 19);
            hashMap.put(blueCloseReefFace, 18);
            hashMap.put(blueCloseRightReefFace, 17);
            hashMap.put(blueFarLeftReefFace, 20);
            hashMap.put(blueFarReefFace, 21);
            hashMap.put(blueFarRightReefFace, 22);
        }
        aKitLog.record("TargetAprilTagID", hashMap.get(targetReefFacePose));

        return hashMap.get(targetReefFacePose);
    }

}

