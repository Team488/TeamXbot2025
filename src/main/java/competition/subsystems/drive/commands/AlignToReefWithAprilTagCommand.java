package competition.subsystems.drive.commands;

import competition.electrical_contract.ElectricalContract;
import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.drive.logic.AlignCameraToAprilTagCalculator;
import competition.subsystems.pose.DriverRelativeCameraValues;
import competition.subsystems.pose.PoseSubsystem;
import competition.subsystems.vision.AprilTagVisionSubsystemExtended;
import xbot.common.subsystems.drive.control_logic.HeadingModule;

import javax.inject.Inject;
import java.util.Arrays;
import java.util.List;

public class AlignToReefWithAprilTagCommand extends AlignToTagGlobalMovementWithCalculator {

    final AprilTagVisionSubsystemExtended aprilTagVisionSubsystem;
    private int cameraToUse;
    private boolean isCameraBackwards;
    private double offsetInInches;
    private boolean isDriverRelative = false;
    private boolean hasCameraFlippedDriverRelative = false;
    private boolean hasSetConfiguration = false;
    private boolean requireExcellentAlignment = true;
    private AlignCameraToAprilTagCalculator.Activity startingActivity = AlignCameraToAprilTagCalculator.Activity.Searching;

    @Inject
    public AlignToReefWithAprilTagCommand(AprilTagVisionSubsystemExtended aprilTagVisionSubsystem, DriveSubsystem drive,
                                          HeadingModule.HeadingModuleFactory headingModuleFactory, PoseSubsystem pose,
                                          ElectricalContract electricalContract,
                                          AlignCameraToAprilTagCalculator.AlignCameraToAprilTagCalculatorFactory calculatorFactory) {
        super(aprilTagVisionSubsystem, drive, headingModuleFactory, pose, electricalContract, calculatorFactory);

        this.aprilTagVisionSubsystem = aprilTagVisionSubsystem;
    }

    public void setConfigurations(int cameraToUse, boolean isCameraBackwards, double offsetInInches, boolean isDriverRelative) {
        this.cameraToUse = cameraToUse;
        this.isCameraBackwards = isCameraBackwards;
        this.offsetInInches = offsetInInches;
        this.isDriverRelative = isDriverRelative;
        this.hasSetConfiguration = true;
    }

    @Override
    public void initialize() {
        log.info("Initializing");
        if (!hasSetConfiguration) {
            cancel();
            return;
        }

        if (isDriverRelative) {
            setDriverRelativeCameraToUse();
        }

        super.setConfigurations(
                cameraToUse,
                aprilTagVisionSubsystem.getTargetAprilTagID(pose.getClosestReefFacePose()),
                isCameraBackwards,
                offsetInInches
        );
        super.initialize();
    }

    private void setDriverRelativeCameraToUse() {
        DriverRelativeCameraValues values = pose.getDriverRelativeCameraToUse(hasCameraFlippedDriverRelative, cameraToUse);
        hasCameraFlippedDriverRelative = values.hasCameraFlippedDriverRelative();
        cameraToUse = values.cameraToUse();
    }
}
