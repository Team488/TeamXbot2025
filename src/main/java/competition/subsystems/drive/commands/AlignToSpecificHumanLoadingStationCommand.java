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
import java.util.function.Supplier;

public class AlignToSpecificHumanLoadingStationCommand extends AlignToTagGlobalMovementWithCalculator {

    private Supplier<Landmarks.CoralStation> stationSupplier;

    @Inject
    public AlignToSpecificHumanLoadingStationCommand(
            AprilTagVisionSubsystemExtended aprilTagVisionSubsystem, DriveSubsystem drive,
            HeadingModule.HeadingModuleFactory headingModuleFactory, PoseSubsystem pose,
            ElectricalContract electricalContract,
            AlignCameraToAprilTagCalculator.AlignCameraToAprilTagCalculatorFactory calculatorFactory) {
        super(aprilTagVisionSubsystem, drive, headingModuleFactory, pose, electricalContract, calculatorFactory);
    }

    public void setTargetCoralStation(Landmarks.CoralStation station) {
        setTargetCoralStationSupplier(() -> station);
    }

    public void setTargetCoralStationSupplier(Supplier<Landmarks.CoralStation> stationSupplier) {
        this.stationSupplier = stationSupplier;
    }

    @Override
    public void initialize() {
        var alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
        this.setConfigurations(
                2,
                Landmarks.getTagIdFromCoralStation(alliance, stationSupplier.get()),
                true,
                1.0,
                AlignCameraToAprilTagCalculator.Activity.ApproachWhileCentering,
                false
        );
        super.initialize();
    }
}
