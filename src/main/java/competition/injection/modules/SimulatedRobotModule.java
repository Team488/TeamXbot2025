package competition.injection.modules;

import javax.inject.Singleton;

import competition.simulation.BaseSimulator;
import competition.simulation.MapleSimulator;
import dagger.Binds;
import dagger.Module;
import xbot.common.subsystems.pose.SimulatedPositionSupplier;

@Module
public abstract class SimulatedRobotModule {
    @Binds
    @Singleton
    public abstract BaseSimulator getSimulator(MapleSimulator impl);
    
    @Binds
    @Singleton
    abstract SimulatedPositionSupplier getSimulatedPositionSupplier(BaseSimulator impl);
}
