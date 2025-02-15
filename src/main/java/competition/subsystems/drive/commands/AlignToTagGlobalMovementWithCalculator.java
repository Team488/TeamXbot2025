package competition.subsystems.drive.commands;

import competition.electrical_contract.ElectricalContract;
import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.pose.PoseSubsystem;
import competition.subsystems.vision.AlignCameraToAprilTagCalculator;
import competition.subsystems.vision.AprilTagVisionSubsystemExtended;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import xbot.common.command.BaseCommand;
import xbot.common.math.XYPair;
import xbot.common.subsystems.drive.control_logic.HeadingModule;

import javax.inject.Inject;

import static edu.wpi.first.units.Units.Inch;
import static edu.wpi.first.units.Units.Inches;

public class AlignToTagGlobalMovementWithCalculator extends BaseCommand {
    final AprilTagVisionSubsystemExtended aprilTagVisionSubsystem;
    final DriveSubsystem drive;
    final HeadingModule.HeadingModuleFactory headingModuleFactory;
    final ElectricalContract electricalContract;
    final PoseSubsystem pose;

    private int targetAprilTagID;
    private int targetCameraID;
    private boolean isCameraBackwards;
    private Distance offset;

    AlignCameraToAprilTagCalculator calculator;

    @Inject
    public AlignToTagGlobalMovementWithCalculator(AprilTagVisionSubsystemExtended aprilTagVisionSubsystem, DriveSubsystem drive,
                                                  HeadingModule.HeadingModuleFactory headingModuleFactory, PoseSubsystem pose,
                                                  ElectricalContract electricalContract) {
        this.aprilTagVisionSubsystem = aprilTagVisionSubsystem;
        this.electricalContract = electricalContract;
        this.drive = drive;
        this.headingModuleFactory = headingModuleFactory;
        this.pose = pose;
        addRequirements(drive);
    }

    public void setConfigurations(int targetCameraID, int targetAprilTagID, boolean isCameraBackwards, int offsetInInches) {
        this.targetCameraID = targetCameraID;
        this.targetAprilTagID = targetAprilTagID;
        this.isCameraBackwards = isCameraBackwards;
        this.offset = Inches.of(offsetInInches);
    }

    @Override
    public void initialize() {
        log.info("Initializing");
        calculator = new AlignCameraToAprilTagCalculator(
                aprilTagVisionSubsystem, drive, electricalContract, pose, headingModuleFactory, targetAprilTagID,
                targetCameraID, offset, isCameraBackwards
        );
    }

    @Override
    public void execute() {
        Pose2d driveValues = calculator.getXYPowersAlignToAprilTag(pose.getCurrentPose2d());
        aKitLog.record("driveValues", driveValues);
        drive.fieldOrientedDrive(
                new XYPair(driveValues.getX(), driveValues.getY()),
                driveValues.getRotation().getDegrees(),
                pose.getCurrentHeading().getDegrees(),
                true);
    }

    @Override
    public boolean isFinished() {
        return calculator.recommendIsFinished();
    }
}