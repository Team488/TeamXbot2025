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

        targetReefFacePose = pose.getClosestReefFacePose();
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
        if (aprilTagVisionSubsystem.reefAprilTagCameraHasCorrectTarget(
                aprilTagVisionSubsystem.getTargetAprilTagID(targetReefFacePose))) {
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
}
