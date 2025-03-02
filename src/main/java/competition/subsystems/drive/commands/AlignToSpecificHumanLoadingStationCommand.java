package competition.subsystems.drive.commands;

import competition.electrical_contract.ElectricalContract;
import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.drive.logic.AlignCameraToAprilTagCalculator;
import competition.subsystems.pose.Landmarks;
import competition.subsystems.pose.PoseSubsystem;
import competition.subsystems.vision.AprilTagVisionSubsystemExtended;
import edu.wpi.first.wpilibj.DriverStation;
import xbot.common.subsystems.drive.control_logic.HeadingModule;

import javax.inject.Inject;

public class AlignToSpecificHumanLoadingStationCommand extends AlignToTagGlobalMovementWithCalculator {

    private Landmarks.CoralStation station;

    @Inject
    public AlignToSpecificHumanLoadingStationCommand(
            AprilTagVisionSubsystemExtended aprilTagVisionSubsystem, DriveSubsystem drive,
            HeadingModule.HeadingModuleFactory headingModuleFactory, PoseSubsystem pose,
            ElectricalContract electricalContract,
            AlignCameraToAprilTagCalculator.AlignCameraToAprilTagCalculatorFactory calculatorFactory) {
        super(aprilTagVisionSubsystem, drive, headingModuleFactory, pose, electricalContract, calculatorFactory);

        station = Landmarks.CoralStation.LEFT;
    }

    public void setCoralStation(Landmarks.CoralStation station) {
        this.station = station;
    }

    @Override
    public void initialize() {
        var alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
        this.setConfigurations(
                2,
                Landmarks.getTagIdFromCoralStation(alliance, station),
                true,
                1.0
        );
        super.initialize();
    }
}
