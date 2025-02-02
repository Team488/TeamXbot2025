package competition.injection.components;

import javax.inject.Singleton;

import competition.injection.modules.CommonBinderModule;
import competition.injection.modules.CommonProviderModule;
import competition.injection.modules.Module2025;
import dagger.Component;
import xbot.common.injection.modules.RealControlsModule;
import xbot.common.injection.modules.RealDevicesModule;
import xbot.common.injection.modules.RobotModule;

@Singleton
@Component(modules = { RobotModule.class, RealDevicesModule.class, RealControlsModule.class,
        Module2025.class, CommonProviderModule.class, CommonBinderModule.class })
public abstract class RobotComponent extends BaseRobotComponent {
    
}
