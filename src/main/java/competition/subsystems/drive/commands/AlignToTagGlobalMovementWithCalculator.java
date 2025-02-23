package competition.subsystems.drive.commands;

import competition.electrical_contract.ElectricalContract;
import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.pose.PoseSubsystem;
import competition.subsystems.drive.logic.AlignCameraToAprilTagCalculator;
import competition.subsystems.vision.AprilTagVisionSubsystemExtended;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Distance;
import xbot.common.command.BaseCommand;
import xbot.common.math.XYPair;
import xbot.common.subsystems.drive.control_logic.HeadingModule;

import javax.inject.Inject;

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

    final AlignCameraToAprilTagCalculator calculator;

    @Inject
    public AlignToTagGlobalMovementWithCalculator(AprilTagVisionSubsystemExtended aprilTagVisionSubsystem, DriveSubsystem drive,
                                                  HeadingModule.HeadingModuleFactory headingModuleFactory, PoseSubsystem pose,
                                                  ElectricalContract electricalContract,
                                                  AlignCameraToAprilTagCalculator.AlignCameraToAprilTagCalculatorFactory calculatorFactory) {
        this.aprilTagVisionSubsystem = aprilTagVisionSubsystem;
        this.electricalContract = electricalContract;
        this.drive = drive;
        this.headingModuleFactory = headingModuleFactory;
        this.pose = pose;
        this.calculator = calculatorFactory.create();
        addRequirements(drive);
    }

    public void setConfigurations(int targetCameraID, int targetAprilTagID, boolean isCameraBackwards, double offsetInInches) {
        this.targetCameraID = targetCameraID;
        this.targetAprilTagID = targetAprilTagID;
        this.isCameraBackwards = isCameraBackwards;
        this.offset = Inches.of(offsetInInches);
    }

    @Override
    public void initialize() {
        log.info("Initializing");
        calculator.reset();
        calculator.configure(targetAprilTagID, targetCameraID, offset, isCameraBackwards);
    }

    @Override
    public void execute() {
        var advice = calculator.getXYPowersAlignToAprilTag(pose.getCurrentPose2d());
        aKitLog.record("driveValues", advice.driveIntent());
        drive.fieldOrientedDrive(
                advice.driveIntent(),
                advice.rotationIntent(),
                pose.getCurrentHeading().getDegrees(),
                true);
    }

    @Override
    public boolean isFinished() {
        return calculator.recommendIsFinished();
    }
}