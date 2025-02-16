package competition.subsystems.drive.commands;

import competition.electrical_contract.ElectricalContract;
import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.pose.PoseSubsystem;
import competition.subsystems.vision.AprilTagVisionSubsystemExtended;
import edu.wpi.first.math.geometry.Pose2d;
import xbot.common.subsystems.drive.control_logic.HeadingModule;

import javax.inject.Inject;

public class AlignToReefWithAprilTagCommand extends AlignToTagGlobalMovementWithCalculator {

    final AprilTagVisionSubsystemExtended aprilTagVisionSubsystem;
    private int cameraToUse;
    private boolean isCameraBackwards;
    private double offsetInInches;

    @Inject
    public AlignToReefWithAprilTagCommand(AprilTagVisionSubsystemExtended aprilTagVisionSubsystem, DriveSubsystem drive,
                                          HeadingModule.HeadingModuleFactory headingModuleFactory, PoseSubsystem pose,
                                          ElectricalContract electricalContract) {
        super(aprilTagVisionSubsystem, drive, headingModuleFactory, pose, electricalContract);

        this.aprilTagVisionSubsystem = aprilTagVisionSubsystem;
    }

    public void setConfigurations(int cameraToUse, boolean isCameraBackwards, double offsetInInches) {
        this.cameraToUse = cameraToUse;
        this.isCameraBackwards = isCameraBackwards;
        this.offsetInInches = offsetInInches;
    }

    @Override
    public void initialize() {
        super.setConfigurations(
                cameraToUse,
                aprilTagVisionSubsystem.getTargetAprilTagID(pose.getClosestReefFacePose()),
                isCameraBackwards,
                offsetInInches
        );
        super.initialize();
    }
}
