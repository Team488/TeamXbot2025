package competition.subsystems.drive.commands;

import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.pose.PoseSubsystem;
import competition.subsystems.vision.MockAprilTagVisionSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import xbot.common.command.BaseCommand;
import xbot.common.math.XYPair;
import xbot.common.subsystems.drive.control_logic.HeadingModule;

import javax.inject.Inject;
import java.util.HashMap;

public class AlignToReefWithAprilTagCommand extends BaseCommand {
    MockAprilTagVisionSubsystem aprilTagVisionSubsystem;
    DriveSubsystem drive;
    HeadingModule headingModule;
    PoseSubsystem pose;

    public static Pose2d BlueCloseReefFace = new Pose2d(3.158, 4.026, Rotation2d.fromDegrees(0));
    public static Pose2d BlueCloseRightReefFace = new Pose2d(3.829, 2.880, Rotation2d.fromDegrees(60));
    public static Pose2d BlueCloseLeftReefFace = new Pose2d(3.829, 5.180, Rotation2d.fromDegrees(-60));
    public static Pose2d BlueBackLeftReefFace = new Pose2d(5.150, 5.180, Rotation2d.fromDegrees(-120));
    public static Pose2d BlueBackReefFace = new Pose2d(5.821, 5.821 , Rotation2d.fromDegrees(-180));
    public static Pose2d BlueBackRightReefFace = new Pose2d(5.150, 2.880, Rotation2d.fromDegrees(120));

    @Inject
    public AlignToReefWithAprilTagCommand(MockAprilTagVisionSubsystem aprilTagVisionSubsystem, DriveSubsystem drive,
                                           HeadingModule.HeadingModuleFactory headingModuleFactory, PoseSubsystem pose) {
        this.aprilTagVisionSubsystem = aprilTagVisionSubsystem;
        this.drive = drive;
        this.headingModule = headingModuleFactory.create(drive.getRotateToHeadingPid());
        this.pose = pose;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        drive.getPositionalPid().reset();
    }

    @Override
    public void execute() {
        Translation2d driveValues = getXYPowersAlignToAprilTag();

        double omega = headingModule.calculateHeadingPower(getClosestReefFacePose().getRotation().getDegrees());

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
        Translation2d aprilTagData = aprilTagVisionSubsystem.getReefAprilTagCameraData();

        double dx = drive.getPositionalPid().calculate(0.5, aprilTagData.getX());
        double dy = drive.getPositionalPid().calculate(0, aprilTagData.getY());

        aKitLog.record("AprilTag X", aprilTagData.getX());
        aKitLog.record("AprilTag Y", aprilTagData.getY());

        return new Translation2d(dx, dy);
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
}
