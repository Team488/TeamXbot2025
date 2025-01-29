package competition.subsystems.drive.commands;

import competition.electrical_contract.ElectricalContract;
import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.pose.Landmarks;
import competition.subsystems.pose.PoseSubsystem;
import competition.subsystems.vision.AprilTagVisionSubsystemExtended;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import xbot.common.command.BaseCommand;
import xbot.common.injection.electrical_contract.CameraInfo;
import xbot.common.math.XYPair;
import xbot.common.subsystems.drive.control_logic.HeadingModule;

import javax.inject.Inject;
import java.util.HashMap;

import static edu.wpi.first.units.Units.Meters;

public class AlignToReefWithAprilTagCommand extends BaseCommand {
    AprilTagVisionSubsystemExtended aprilTagVisionSubsystem;
    DriveSubsystem drive;
    HeadingModule headingModule;
    PoseSubsystem pose;

    Pose2d blueCloseReefFace = Landmarks.BlueCloseAlgae;
    Pose2d blueCloseRightReefFace = Landmarks.BlueCloseRightAlgae;
    Pose2d blueCloseLeftReefFace = Landmarks.BlueCloseLeftAlgae;
    Pose2d blueFarLeftReefFace = Landmarks.BlueFarLeftAlgae;
    Pose2d blueFarReefFace = Landmarks.BlueFarAlgae;
    Pose2d blueFarRightReefFace = Landmarks.BlueFarRightAlgae;

    public Distance cameraXOffset;
    public Pose2d targetReefFacePose;

    @Inject
    public AlignToReefWithAprilTagCommand(AprilTagVisionSubsystemExtended aprilTagVisionSubsystem, DriveSubsystem drive,
                                          HeadingModule.HeadingModuleFactory headingModuleFactory, PoseSubsystem pose,
                                          ElectricalContract electricalContract) {
        this.aprilTagVisionSubsystem = aprilTagVisionSubsystem;
        this.drive = drive;
        this.headingModule = headingModuleFactory.create(drive.getRotateToHeadingPid());
        this.pose = pose;
        addRequirements(drive);
        CameraInfo[] cameraInfos = electricalContract.getCameraInfo();
        this.cameraXOffset = cameraInfos[0].position().getMeasureX();
    }

    @Override
    public void initialize() {
        drive.getPositionalPid().reset();

        targetReefFacePose = getClosestReefFacePose();
        aKitLog.record("TargetReefFace", targetReefFacePose);
    }

    @Override
    public void execute() {
        Translation2d driveValues = getXYPowersAlignToAprilTag();

        double omega = headingModule.calculateHeadingPower(targetReefFacePose.getRotation().getDegrees());

        drive.move(new XYPair(-driveValues.getX(), -driveValues.getY()), omega);

        aKitLog.record("dx power", driveValues.getX());
        aKitLog.record("dy power", driveValues.getY());
        aKitLog.record("omega", omega);

        aKitLog.record("camera count", aprilTagVisionSubsystem.getCameraCount());
    }

    @Override
    public boolean isFinished() {
        return drive.getPositionalPid().isOnTarget() && drive.getRotateToHeadingPid().isOnTarget();
    }

    public Translation2d getXYPowersAlignToAprilTag() {
        // only calculate powers if we have target april tag in sight
        if (aprilTagVisionSubsystem.reefAprilTagCameraHasCorrectTarget(getTargetAprilTagID())) {
            Translation2d aprilTagData = aprilTagVisionSubsystem.getReefAprilTagCameraData();

            double dx = drive.getPositionalPid().calculate(cameraXOffset.in(Meters), aprilTagData.getX());
            double dy = drive.getPositionalPid().calculate(0, aprilTagData.getY());

            aKitLog.record("AprilTag X", aprilTagData.getX());
            aKitLog.record("AprilTag Y", aprilTagData.getY());
            return new Translation2d(dx, dy);
        }
        cancel();
        return new Translation2d(0,0);
    }

    private Pose2d getClosestReefFacePose() {
        Pose2d currentPose = pose.getCurrentPose2d();

        double closeDistance = PoseSubsystem.convertBlueToRedIfNeeded(
                blueCloseReefFace).getTranslation().getDistance(currentPose.getTranslation());
        double closeLeftDistance = PoseSubsystem.convertBlueToRedIfNeeded(
                blueCloseLeftReefFace).getTranslation().getDistance(currentPose.getTranslation());
        double closeRightDistance = PoseSubsystem.convertBlueToRedIfNeeded(
                blueCloseRightReefFace).getTranslation().getDistance(currentPose.getTranslation());
        double backLeftDistance = PoseSubsystem.convertBlueToRedIfNeeded(
                blueFarLeftReefFace).getTranslation().getDistance(currentPose.getTranslation());
        double backDistance = PoseSubsystem.convertBlueToRedIfNeeded(
                blueFarReefFace).getTranslation().getDistance(currentPose.getTranslation());
        double backRightDistance = PoseSubsystem.convertBlueToRedIfNeeded(
                blueFarRightReefFace).getTranslation().getDistance(currentPose.getTranslation());

        HashMap<Double, Pose2d> hashMap = new HashMap<>();
        hashMap.put(closeLeftDistance, PoseSubsystem.convertBlueToRedIfNeeded(blueCloseLeftReefFace));
        hashMap.put(closeDistance, PoseSubsystem.convertBlueToRedIfNeeded(blueCloseReefFace));
        hashMap.put(closeRightDistance, PoseSubsystem.convertBlueToRedIfNeeded(blueCloseRightReefFace));
        hashMap.put(backLeftDistance, PoseSubsystem.convertBlueToRedIfNeeded(blueFarLeftReefFace));
        hashMap.put(backDistance, PoseSubsystem.convertBlueToRedIfNeeded(blueFarReefFace));
        hashMap.put(backRightDistance, PoseSubsystem.convertBlueToRedIfNeeded(blueFarRightReefFace));

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
