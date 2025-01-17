package competition.injection.components;

import competition.injection.modules.CommonModule;
import competition.injection.modules.Module2024;
import dagger.Component;
import xbot.common.injection.modules.RealControlsModule;
import xbot.common.injection.modules.RealDevicesModule;
import xbot.common.injection.modules.RobotModule;

import javax.inject.Singleton;

@Singleton
@Component(modules = { RobotModule.class, RealDevicesModule.class, RealControlsModule.class,
        Module2024.class, CommonModule.class})
public abstract class RobotComponent2024 extends BaseRobotComponent {
    
}
