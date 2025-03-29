package competition.subsystems.drive.commands;

import competition.electrical_contract.ElectricalContract;
import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.pose.PoseSubsystem;
import competition.subsystems.drive.logic.AlignCameraToAprilTagCalculator;
import competition.subsystems.vision.AprilTagVisionSubsystemExtended;
import edu.wpi.first.units.measure.Distance;
import xbot.common.command.BaseCommand;
import xbot.common.controls.sensors.XTimer;
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
    private boolean hasSetConfiguration = false;
    private AlignCameraToAprilTagCalculator.Activity startingActivity = AlignCameraToAprilTagCalculator.Activity.Searching;
    private boolean requireExcellentAlignment = true;
    private boolean useCreeperAlignment = true;

    final AlignCameraToAprilTagCalculator calculator;
    private double startTime = 0;

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
        setConfigurations(
                targetCameraID,
                targetAprilTagID,
                isCameraBackwards,
                offsetInInches,
                AlignCameraToAprilTagCalculator.Activity.Searching,
                true,
                true);
    }

    public void setConfigurations(int targetCameraID, int targetAprilTagID, boolean isCameraBackwards, double offsetInInches,
                                  AlignCameraToAprilTagCalculator.Activity startingActivity,
                                  boolean requireExcellentAlignment, boolean useCreeperAlignment) {
        this.targetCameraID = targetCameraID;
        this.targetAprilTagID = targetAprilTagID;
        this.isCameraBackwards = isCameraBackwards;
        this.offset = Inches.of(offsetInInches);
        this.hasSetConfiguration = true;
        this.startingActivity = startingActivity;
        this.requireExcellentAlignment = requireExcellentAlignment;
        this.useCreeperAlignment = useCreeperAlignment;
    }

    @Override
    public void initialize() {
        log.info("Initializing");
        if (!hasSetConfiguration) {
            cancel();
            return;
        }

        calculator.configureAndReset(targetAprilTagID, targetCameraID, offset,
                isCameraBackwards, startingActivity, requireExcellentAlignment, useCreeperAlignment);
        pose.setPreferOdometryToVision(true);
        startTime = XTimer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        if(!hasSetConfiguration) {
            return;
        }
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
        return hasSetConfiguration && calculator.recommendIsFinished();
    }

    @Override
    public void end(boolean interrupted) {
        pose.setPreferOdometryToVision(false);
        drive.stop();
        var duration = XTimer.getFPGATimestamp() - startTime;
        aKitLog.record("align duration", duration);
        log.info("align duration: ", duration);
    }
}