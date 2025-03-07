package competition.subsystems.drive.commands;

import competition.electrical_contract.ElectricalContract;
import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.drive.logic.AlignCameraToAprilTagCalculator;
import competition.subsystems.pose.PoseSubsystem;
import competition.subsystems.vision.AprilTagVisionSubsystemExtended;
import xbot.common.subsystems.drive.control_logic.HeadingModule;

import javax.inject.Inject;

public class AlignToNearestCoralStationCommand extends AlignToSpecificHumanLoadingStationCommand{

    @Inject
    public AlignToNearestCoralStationCommand(AprilTagVisionSubsystemExtended aprilTagVisionSubsystem,
                                             DriveSubsystem drive, HeadingModule.HeadingModuleFactory headingModuleFactory,
                                             PoseSubsystem pose, ElectricalContract electricalContract,
                                             AlignCameraToAprilTagCalculator.AlignCameraToAprilTagCalculatorFactory calculatorFactory) {
        super(aprilTagVisionSubsystem, drive, headingModuleFactory, pose, electricalContract, calculatorFactory);
    }

    @Override
    public void initialize() {
        super.initialize();
        setCoralStation(pose.getClosestCoralStation());
        super.initialize();
    }
}