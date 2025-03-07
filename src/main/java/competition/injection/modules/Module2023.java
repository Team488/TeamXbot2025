package competition.injection.modules;

import competition.electrical_contract.Contract2023;
import competition.electrical_contract.ElectricalContract;
import competition.simulation.BaseSimulator;
import competition.simulation.NoopSimulator;
import dagger.Binds;
import dagger.Module;

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
