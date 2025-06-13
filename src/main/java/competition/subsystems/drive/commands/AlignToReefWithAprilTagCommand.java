package competition.subsystems.drive.commands;

import competition.electrical_contract.ElectricalContract;
import competition.operator_interface.OperatorInterface;
import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.drive.logic.AlignCameraToAprilTagCalculator;
import competition.subsystems.pose.DriverRelativeCameraValues;
import competition.subsystems.pose.PoseSubsystem;
import competition.subsystems.vision.AprilTagVisionSubsystemExtended;
import edu.wpi.first.networktables.BooleanEntry;
import xbot.common.subsystems.drive.control_logic.HeadingModule;

import javax.inject.Inject;
import java.util.Arrays;
import java.util.List;

public class AlignToReefWithAprilTagCommand extends AlignToTagGlobalMovementWithCalculator {

    final AprilTagVisionSubsystemExtended aprilTagVisionSubsystem;
    final OperatorInterface oi;
    private int cameraToUse;
    private boolean isCameraBackwards;
    private double offsetInInches;
    private boolean isDriverRelative = false;
    private boolean hasCameraFlippedDriverRelative = false;
    private boolean hasSetConfiguration = false;
    private AlignCameraToAprilTagCalculator.Activity startingActivity = AlignCameraToAprilTagCalculator.Activity.Searching;
    private boolean enableRetries = true;

    @Inject
    public AlignToReefWithAprilTagCommand(AprilTagVisionSubsystemExtended aprilTagVisionSubsystem, DriveSubsystem drive,
                                          HeadingModule.HeadingModuleFactory headingModuleFactory, PoseSubsystem pose,
                                          ElectricalContract electricalContract,
                                          AlignCameraToAprilTagCalculator.AlignCameraToAprilTagCalculatorFactory calculatorFactory,
                                          OperatorInterface oi) {
        super(aprilTagVisionSubsystem, drive, headingModuleFactory, pose, electricalContract, calculatorFactory);

        this.aprilTagVisionSubsystem = aprilTagVisionSubsystem;
        this.oi = oi;
    }

    public void setConfigurations(int cameraToUse, boolean isCameraBackwards, double offsetInInches, boolean isDriverRelative) {
        setConfigurations(cameraToUse, isCameraBackwards, offsetInInches, isDriverRelative,
                AlignCameraToAprilTagCalculator.Activity.Searching, true);
    }

    public void setConfigurations(int cameraToUse, boolean isCameraBackwards, double offsetInInches, boolean isDriverRelative,
                                  AlignCameraToAprilTagCalculator.Activity startingActivity, boolean enableRetries) {
        this.cameraToUse = cameraToUse;
        this.isCameraBackwards = isCameraBackwards;
        this.offsetInInches = offsetInInches;
        this.isDriverRelative = isDriverRelative;
        this.startingActivity = startingActivity;
        this.enableRetries = enableRetries;
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

        if (!aprilTagVisionSubsystem.isCameraConnected(cameraToUse)) {
            // if the camera isn't connected, we can't really do this
            // rumble to notify driver
            oi.driverGamepad.getRumbleManager().rumbleGamepad(1, 1);
        } else {
            super.setConfigurations(
                    cameraToUse,
                    aprilTagVisionSubsystem.getTargetAprilTagID(pose.getReefFaceFromAngle()),
                    isCameraBackwards,
                    offsetInInches,
                    startingActivity,
                    enableRetries
            );
    
            super.initialize();
        }
    }

    private void setDriverRelativeCameraToUse() {
        DriverRelativeCameraValues values = pose.getDriverRelativeCameraToUse(hasCameraFlippedDriverRelative, cameraToUse);
        hasCameraFlippedDriverRelative = values.hasCameraFlippedDriverRelative();
        cameraToUse = values.cameraToUse();
    }
}
