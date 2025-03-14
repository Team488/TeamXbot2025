package competition.injection.modules;

import javax.inject.Singleton;

import competition.electrical_contract.Contract2025;
import competition.electrical_contract.ElectricalContract;
import competition.simulation.BaseSimulator;
import competition.simulation.NoopSimulator;
import dagger.Binds;
import dagger.Module;

@Module
public abstract class Module2025 {
    @Binds
    @Singleton
    public abstract ElectricalContract getElectricalContract(Contract2025 impl);

    @Binds
    @Singleton
    public abstract BaseSimulator getSimulator(NoopSimulator impl);
}
