package competition.injection.components;

import javax.inject.Singleton;

import competition.injection.modules.CommonBinderModule;
import competition.injection.modules.CommonProviderModule;
import competition.injection.modules.UnitTestRobotModule;
import dagger.Component;
import xbot.common.injection.modules.MockControlsModule;
import xbot.common.injection.modules.MockDevicesModule;
import xbot.common.injection.modules.UnitTestModule;

@Singleton
@Component(modules = { UnitTestModule.class, MockDevicesModule.class, MockControlsModule.class,
        CommonProviderModule.class, CommonBinderModule.class, UnitTestRobotModule.class})
public abstract class CompetitionTestComponent extends BaseRobotComponent {
    
}
