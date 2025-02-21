package competition.injection.modules;

import competition.electrical_contract.Contract2023;
import competition.electrical_contract.ElectricalContract;
import competition.simulation.BaseSimulator;
import competition.simulation.NoopSimulator;
import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.pose.PoseSubsystem;
import competition.subsystems.vision.AprilTagVisionSubsystemExtended;
import dagger.Binds;
import dagger.Module;
import xbot.common.injection.electrical_contract.XCameraElectricalContract;
import xbot.common.injection.electrical_contract.XSwerveDriveElectricalContract;
import xbot.common.subsystems.drive.BaseDriveSubsystem;
import xbot.common.subsystems.drive.BaseSwerveDriveSubsystem;
import xbot.common.subsystems.pose.BasePoseSubsystem;
import xbot.common.subsystems.vision.AprilTagVisionSubsystem;

import javax.inject.Singleton;

@Module
public abstract class Module2023 {
    @Binds
    @Singleton
    public abstract ElectricalContract getElectricalContract(Contract2023 impl);

    @Binds
    @Singleton
    public abstract BaseSimulator getSimulator(NoopSimulator impl);
}
