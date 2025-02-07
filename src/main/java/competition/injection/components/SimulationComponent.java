package competition.injection.components;

import javax.inject.Singleton;

import competition.electrical_contract.ElectricalContract;
import competition.injection.modules.CommonBinderModule;
import competition.injection.modules.CommonProviderModule;
import competition.injection.modules.SimulatedRobotModule;
import dagger.BindsInstance;
import dagger.Component;
import xbot.common.injection.modules.MockDevicesModule;
import xbot.common.injection.modules.RealControlsModule;
import xbot.common.injection.modules.SimulationModule;

@Singleton
@Component(modules = { SimulationModule.class, MockDevicesModule.class, RealControlsModule.class,
        SimulatedRobotModule.class, CommonProviderModule.class, CommonBinderModule.class })
public abstract class SimulationComponent extends BaseRobotComponent {
    @Component.Builder
    public abstract static class Builder {
        @BindsInstance
        public abstract Builder electricalContract(ElectricalContract contract);
        public abstract BaseRobotComponent build();
    }
}
