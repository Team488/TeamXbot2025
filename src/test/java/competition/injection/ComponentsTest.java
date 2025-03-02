package competition.injection;

import competition.injection.components.DaggerRobotComponent;
import competition.injection.components.DaggerRobotComponent2023;
import competition.injection.components.DaggerRobotComponent2024;
import competition.simulation.MapleSimulator;
import competition.simulation.MapleSimulator_Factory;
import org.junit.Test;
import xbot.common.controls.actuators.mock_adapters.MockCANMotorController;
import xbot.common.controls.actuators.mock_adapters.MockCANMotorController_Factory;
import xbot.common.injection.components.BaseComponent;

import javax.inject.Provider;
import java.lang.reflect.Field;
import java.lang.reflect.ParameterizedType;
import java.lang.reflect.Type;
import java.util.HashSet;
import java.util.Set;

import static org.junit.Assert.fail;

public class ComponentsTest {
    @Test
    public void testRealRobotsDoNotHaveMockDevices() {
        Set<Type> forbiddenTypes = new HashSet<>();
        forbiddenTypes.add(MockCANMotorController_Factory.class);
        forbiddenTypes.add(MockCANMotorController.MockCANMotorControllerFactory.class);
        forbiddenTypes.add(MapleSimulator_Factory.class);
        forbiddenTypes.add(MapleSimulator.class);

        for (var component : new BaseComponent[]{
                DaggerRobotComponent.create(),
                DaggerRobotComponent2024.create(),
                DaggerRobotComponent2023.create()
        }) {
            Class<?> clazz = component.getClass();
            Field[] fields = clazz.getDeclaredFields();
            for (Field field : fields) {
                field.setAccessible(true);
                if (forbiddenTypes.contains(field.getType())) {
                    fail("Unexpected mock device found in component: " + field.getType().getName() + " in " + clazz.getName());
                }

                if (field.getType() == Provider.class) {
                    var providerFieldType = (ParameterizedType)field.getGenericType();
                    Type genericParamType = providerFieldType.getActualTypeArguments()[0];
                    if (forbiddenTypes.contains(genericParamType)) {
                        fail("Unexpected mock device found in component: " + genericParamType.getTypeName() + " in " + clazz.getName());
                    }
                }
            }
        }
    }
}
