package competition.injection.modules;

import competition.electrical_contract.ElectricalContract;
import competition.injection.RobotOverride;
import competition.simulation.BaseSimulator;
import competition.simulation.NoopSimulator;
import competition.electrical_contract.Contract2024;
import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.pose.PoseSubsystem;
import competition.subsystems.vision.AprilTagVisionSubsystemExtended;
import dagger.Binds;
import dagger.Module;
import xbot.common.controls.sensors.XGyro;
import xbot.common.controls.sensors.wpi_adapters.InertialMeasurementUnitAdapter;
import xbot.common.injection.electrical_contract.XCameraElectricalContract;
import xbot.common.injection.electrical_contract.XSwerveDriveElectricalContract;
import xbot.common.subsystems.drive.BaseDriveSubsystem;
import xbot.common.subsystems.drive.BaseSwerveDriveSubsystem;
import xbot.common.subsystems.pose.BasePoseSubsystem;
import xbot.common.subsystems.vision.AprilTagVisionSubsystem;

import javax.inject.Singleton;

@Module
public abstract class Module2024 {
    @Binds
    @Singleton
    public abstract ElectricalContract getElectricalContract(Contract2024 impl);

    @Binds
    @Singleton
    public abstract BaseSimulator getSimulator(NoopSimulator impl);

    @Binds
    @Singleton
    @RobotOverride
    public abstract XGyro.XGyroFactory getGyroFactory(InertialMeasurementUnitAdapter.InertialMeasurementUnitAdapterFactory impl);
}
