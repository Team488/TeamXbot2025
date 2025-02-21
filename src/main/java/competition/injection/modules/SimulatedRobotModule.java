package competition.injection.modules;

import javax.inject.Singleton;

import competition.electrical_contract.ElectricalContract;
import competition.electrical_contract.UnitTestContract2025;
import competition.simulation.BaseSimulator;
import competition.simulation.MapleSimulator;
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
import xbot.common.subsystems.pose.SimulatedPositionSupplier;
import xbot.common.subsystems.vision.AprilTagVisionSubsystem;

@Module
public abstract class SimulatedRobotModule {
    @Binds
    @Singleton
    public abstract BaseSimulator getSimulator(MapleSimulator impl);
    
    @Binds
    @Singleton
    abstract SimulatedPositionSupplier getSimulatedPositionSupplier(BaseSimulator impl);
}
