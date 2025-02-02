package competition.injection.modules;

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
import xbot.common.subsystems.vision.AprilTagVisionSubsystem;

import javax.inject.Singleton;

@Module
public abstract class UnitTestRobotModule {
    @Binds
    @Singleton
    public abstract ElectricalContract getElectricalContract(UnitTestContract2025 impl);

    @Binds
    @Singleton
    public abstract BaseSimulator getSimulator(MapleSimulator impl);
}
