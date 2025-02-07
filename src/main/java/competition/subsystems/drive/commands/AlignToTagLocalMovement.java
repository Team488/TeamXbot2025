package competition.subsystems.drive.commands;

import competition.electrical_contract.ElectricalContract;
import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.pose.PoseSubsystem;
import competition.subsystems.vision.AprilTagVisionSubsystemExtended;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import xbot.common.command.BaseCommand;
import xbot.common.injection.electrical_contract.CameraInfo;
import xbot.common.math.XYPair;
import xbot.common.subsystems.drive.control_logic.HeadingModule;

import javax.inject.Inject;

import static edu.wpi.first.units.Units.Meters;

public class AlignToTagLocalMovement extends BaseCommand {
    AprilTagVisionSubsystemExtended aprilTagVisionSubsystem;
    DriveSubsystem drive;
    HeadingModule headingModule;
    PoseSubsystem pose;
    public Distance cameraXOffset;
    public Pose2d targetReefFacePose;

    private final Translation2d alignmentPointOffset;
    private int targetAprilTagID;

    @Inject
    public AlignToTagLocalMovement(AprilTagVisionSubsystemExtended aprilTagVisionSubsystem, DriveSubsystem drive,
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

    public void setAprilTagTaret(int targetAprilTagID) {
        this.targetAprilTagID = targetAprilTagID;
    }

    double headingAtCommandStart = 0;

    @Override
    public void initialize() {
        log.info("Initializing");
        drive.getPositionalPid().reset();
        headingAtCommandStart = pose.getCurrentPose2d().getRotation().getDegrees();
    }

    @Override
    public void execute() {
        Translation2d driveValues = getXYPowersAlignToAprilTag();

        double omega = headingModule.calculateHeadingPower(headingAtCommandStart);

        aKitLog.record("PowerVector", driveValues);
        aKitLog.record("omega", omega);
        drive.move(new XYPair(driveValues.getX(), driveValues.getY()), omega);
    }

    @Override
    public boolean isFinished() {
        return drive.getPositionalPid().isOnTarget() && drive.getRotateToHeadingPid().isOnTarget();
    }

    public Translation2d getXYPowersAlignToAprilTag() {
        // only calculate powers if we have the specific target april tag in sight
        if (aprilTagVisionSubsystem.reefAprilTagCameraHasCorrectTarget(targetAprilTagID)) {
            Translation2d aprilTagData = aprilTagVisionSubsystem.getReefAprilTagCameraData();

            Translation2d goalVector = aprilTagData.minus(alignmentPointOffset);
            Translation2d powerVector = drive.getPowerForRelativePositionChange(goalVector);

            aKitLog.record("AprilTag X", aprilTagData.getX());
            aKitLog.record("AprilTag Y", aprilTagData.getY());
            return powerVector;
        } else {
            // Can't see a tag. Stop moving.
            return new Translation2d(0,0);
        }


    }
}