package competition.injection.modules;

import competition.electrical_contract.ElectricalContract;
import competition.simulation.BaseSimulator;
import competition.simulation.NoopSimulator;
import competition.electrical_contract.Contract2024;
import dagger.Binds;
import dagger.Module;

import javax.inject.Singleton;

@Module
public abstract class Module2024 {
    @Binds
    @Singleton
    public abstract ElectricalContract getElectricalContract(Contract2024 impl);

    @Binds
    @Singleton
    public abstract BaseSimulator getSimulator(NoopSimulator impl);
}
