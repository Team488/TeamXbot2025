package competition.subsystems.drive.commands;

import competition.electrical_contract.ElectricalContract;
import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.pose.Landmarks;
import competition.subsystems.pose.PoseSubsystem;
import competition.subsystems.vision.AprilTagVisionSubsystemExtended;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import xbot.common.command.BaseCommand;
import xbot.common.injection.electrical_contract.CameraInfo;
import xbot.common.math.XYPair;
import xbot.common.subsystems.drive.control_logic.HeadingModule;

import javax.inject.Inject;
import java.util.HashMap;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

public class AlignToReefWithAprilTagCommand extends BaseCommand {
    AprilTagVisionSubsystemExtended aprilTagVisionSubsystem;
    DriveSubsystem drive;
    HeadingModule headingModule;
    PoseSubsystem pose;
    public Distance cameraXOffset;
    public Pose2d targetReefFacePose;
    double xOffset;
    double yOffset;

    private final Translation2d alignmentPointOffset;

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

        this.alignmentPointOffset = new Translation2d(
                electricalContract.getDistanceFromCenterToOuterBumperX()
                        .minus(cameraInfos[0].position().getMeasureX()),
                Meters.zero()
        );

    }

    @Override
    public void initialize() {
        log.info("Initializing");
        drive.getPositionalPid().reset();

        targetReefFacePose = pose.getClosestReefFacePose();
        aKitLog.record("TargetReefFace", targetReefFacePose);
    }

    @Override
    public void execute() {
        Translation2d driveValues = getXYPowersAlignToAprilTag();

        double omega = headingModule.calculateHeadingPower(targetReefFacePose.getRotation().getDegrees());

        aKitLog.record("PowerVector", driveValues);
        aKitLog.record("omega", omega);
        drive.move(new XYPair(driveValues.getX(), driveValues.getY()), omega);
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

            Translation2d goalVector = aprilTagData.minus(alignmentPointOffset);
            Translation2d goalVectorWithOffsets = new Translation2d(goalVector.getX() + xOffset,
                    goalVector.getY() + yOffset);
            Translation2d powerVector = drive.getPowerForRelativePositionChange(goalVectorWithOffsets);

            aKitLog.record("AprilTag X", aprilTagData.getX());
            aKitLog.record("AprilTag Y", aprilTagData.getY());
            return powerVector;
        }

        log.info("April tag not in sight, cancelling");
        cancel();
        return new Translation2d(0,0);
    }

    public void setXOffset(Landmarks.CoralLevel branchLevel) {
        // TODO: add switch case to set offset depending on branch level, this is only needed if level scoring positions are different
    }

    public void setYOFFset(Landmarks.Branch branch) {
        // TODO: add if else to set y offset depending on branch
    }
}
