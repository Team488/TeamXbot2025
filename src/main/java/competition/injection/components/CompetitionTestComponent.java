package competition.injection.components;

import javax.inject.Singleton;

import competition.electrical_contract.ElectricalContract;
import competition.injection.modules.CommonModule;
import competition.injection.modules.UnitTestRobotModule;
import dagger.BindsInstance;
import dagger.Component;
import xbot.common.injection.modules.MockControlsModule;
import xbot.common.injection.modules.MockDevicesModule;
import xbot.common.injection.modules.UnitTestModule;

@Singleton
@Component(modules = { UnitTestModule.class, MockDevicesModule.class, MockControlsModule.class,
        CommonModule.class, UnitTestRobotModule.class})
public abstract class CompetitionTestComponent extends BaseRobotComponent {
    @Component.Builder
    public abstract static class Builder {
        @BindsInstance
        public abstract CompetitionTestComponent.Builder electricalContract(ElectricalContract contract);

        public abstract BaseRobotComponent build();
    }
}
