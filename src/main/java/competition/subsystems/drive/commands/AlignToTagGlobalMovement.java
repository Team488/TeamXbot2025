package competition.subsystems.drive.commands;

import competition.electrical_contract.ElectricalContract;
import competition.subsystems.drive.DrivePowerCalculator;
import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.pose.PoseSubsystem;
import competition.subsystems.vision.AprilTagVisionSubsystemExtended;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import xbot.common.command.BaseCommand;
import xbot.common.injection.electrical_contract.CameraInfo;
import xbot.common.math.XYPair;
import xbot.common.subsystems.drive.control_logic.HeadingModule;

import javax.inject.Inject;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

public class AlignToTagGlobalMovement extends BaseCommand {
    AprilTagVisionSubsystemExtended aprilTagVisionSubsystem;
    DriveSubsystem drive;
    HeadingModule headingModule;
    PoseSubsystem pose;
    public Distance cameraXOffset;
    public Pose2d targetReefFacePose;

    private final Translation2d alignmentPointOffset;
    private int targetAprilTagID;

    private enum TagAcquisitionState {
        NeverSeen,
        LockedOn,
        Lost
    }

    private Translation2d driveTarget;

    private TagAcquisitionState tagAcquisitionState;

    @Inject
    public AlignToTagGlobalMovement(AprilTagVisionSubsystemExtended aprilTagVisionSubsystem, DriveSubsystem drive,
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
                        .minus(cameraInfos[0].position().getMeasureX()).plus(Inches.of(24)),
                Meters.zero()
        );
    }

    public void setAprilTagTarget(int targetAprilTagID) {
        this.targetAprilTagID = targetAprilTagID;
    }

    double headingAtCommandStart = 0;

    @Override
    public void initialize() {
        log.info("Initializing");
        drive.getPositionalPid().reset();
        headingAtCommandStart = pose.getCurrentPose2d().getRotation().getDegrees();
        tagAcquisitionState = TagAcquisitionState.NeverSeen;
        driveTarget = new Translation2d(0,0);
    }

    @Override
    public void execute() {
        Translation2d driveValues = getXYPowersAlignToAprilTag();
        double omega = headingModule.calculateHeadingPower(headingAtCommandStart);
        drive.fieldOrientedDrive(
                new XYPair(driveValues.getX(), driveValues.getY()),
                omega,
                pose.getCurrentHeading().getDegrees(),
                true);
    }

    @Override
    public boolean isFinished() {
        return drive.getPositionalPid().isOnTarget()
                && drive.getRotateToHeadingPid().isOnTarget()
                && tagAcquisitionState != TagAcquisitionState.NeverSeen;
    }

    public Translation2d getXYPowersAlignToAprilTag() {
        var currentPose = pose.getCurrentPose2d();
        // Try to get a useful point
        if (aprilTagVisionSubsystem.reefAprilTagCameraHasCorrectTarget(targetAprilTagID)) {
            tagAcquisitionState = TagAcquisitionState.LockedOn;
            Translation2d aprilTagData = aprilTagVisionSubsystem.getReefAprilTagCameraData();
            // This transform will always be at rotation 0, since in its own frame, the robot is
            // always facing forward.


            Transform2d relativeGoalTransform =
                    new Transform2d(aprilTagData.minus(alignmentPointOffset), new Rotation2d());
            // Move from robot-relative frame to field frame
            driveTarget = currentPose.transformBy(relativeGoalTransform).getTranslation();

        } else {
            if (tagAcquisitionState == TagAcquisitionState.LockedOn) {
                tagAcquisitionState = TagAcquisitionState.Lost;
            }
        }

        Translation2d powers = switch (tagAcquisitionState) {
            case LockedOn, Lost -> DrivePowerCalculator.getPowerToAchieveFieldPosition(
                    currentPose.getTranslation(), driveTarget, drive.getPositionalPid());
            default ->
                // How did you get here?
                new Translation2d(0, 0);
        };

        aKitLog.record("DriveTarget", driveTarget);
        aKitLog.record("TagAcquisitionState", tagAcquisitionState);
        return powers;
    }
}