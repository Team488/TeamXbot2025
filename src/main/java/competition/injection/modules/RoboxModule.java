package competition.injection.modules;

import competition.electrical_contract.ElectricalContract;
import competition.electrical_contract.RoboxContract;
import competition.simulation.BaseSimulator;
import competition.simulation.NoopSimulator;
import dagger.Binds;
import dagger.Module;

import javax.inject.Singleton;

@Module
public abstract class RoboxModule {
    @Binds
    @Singleton
    public abstract ElectricalContract getElectricalContract(RoboxContract impl);

    @Binds
    @Singleton
    public abstract BaseSimulator getSimulator(NoopSimulator impl);
}
