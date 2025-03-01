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

    @Inject
    public AlignToReefWithAprilTagCommand(AprilTagVisionSubsystemExtended aprilTagVisionSubsystem, DriveSubsystem drive,
                                          HeadingModule.HeadingModuleFactory headingModuleFactory, PoseSubsystem pose,
                                          ElectricalContract electricalContract,
                                          AlignCameraToAprilTagCalculator.AlignCameraToAprilTagCalculatorFactory calculatorFactory) {
        super(aprilTagVisionSubsystem, drive, headingModuleFactory, pose, electricalContract, calculatorFactory);

        this.aprilTagVisionSubsystem = aprilTagVisionSubsystem;
    }

    public void setConfigurations(int cameraToUse, boolean isCameraBackwards, double offsetInInches) {
        this.cameraToUse = cameraToUse;
        this.isCameraBackwards = isCameraBackwards;
        this.offsetInInches = offsetInInches;
    }

    @Override
    public void initialize() {
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

    public void setDriverRelative(boolean isEnabled) {
        this.isDriverRelative = isEnabled;
    }

    private void setDriverRelativeCameraToUse() {
        DriverRelativeCameraValues values = pose.getDriverRelativeCameraToUse(hasCameraFlippedDriverRelative, cameraToUse);
        hasCameraFlippedDriverRelative = values.hasCameraFlippedDriverRelative();
        cameraToUse = values.cameraToUse();
    }
}
