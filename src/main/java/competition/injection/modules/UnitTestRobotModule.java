package competition.injection.modules;

import competition.simulation.BaseSimulator;
import competition.simulation.MapleSimulator;
import dagger.Binds;
import dagger.Module;

import javax.inject.Singleton;

@Module
public abstract class UnitTestRobotModule {
    @Binds
    @Singleton
    public abstract BaseSimulator getSimulator(MapleSimulator impl);
}
