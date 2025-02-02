package competition.subsystems;

import competition.BaseCompetitionTest;
import competition.subsystems.drive.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.junit.Test;
import org.mockito.Mockito;
import xbot.common.command.BaseSubsystem;
import xbot.common.controls.actuators.XCANMotorController;
import xbot.common.injection.BaseWPITest;

import java.lang.reflect.Field;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.List;

import static org.junit.Assert.assertNotEquals;
import static org.junit.Assert.fail;

public class GenericSubsystemTest extends BaseCompetitionTest {

    @Test
    public void testAllSubsystemPeriodic() throws Exception {
        // Find all subsystem implementations via reflection
        List<Object> subsystems = new ArrayList<>();
        for (var injectorMethod : getInjectorComponent().getClass().getMethods()) {
            if (injectorMethod.getParameterCount() != 0) {
                continue;
            }
            if (BaseSubsystem.class.isAssignableFrom(injectorMethod.getReturnType())) {
                injectorMethod.setAccessible(true);
                subsystems.add(injectorMethod.invoke(getInjectorComponent()));
            }
        }
        assertNotEquals(0, subsystems.size());

        for (Object subsystem : subsystems) {
            // Run each subsystem refreshDataFrame method
            Method refreshDataFrameMethod = subsystem.getClass().getMethod("refreshDataFrame");
            refreshDataFrameMethod.setAccessible(true);
            try {
                refreshDataFrameMethod.invoke(subsystem);
            } catch (Exception e) {
                fail("Subsystem " + subsystem.getClass().getName() + " failed to call refreshDataFrame:\n" + e);
            }

            // Run each subsystem periodic method
            Method periodicMethod = subsystem.getClass().getMethod("periodic");
            periodicMethod.setAccessible(true);
            try {
                periodicMethod.invoke(subsystem);
            } catch (Exception e) {
                fail("Subsystem " + subsystem.getClass().getName() + " failed to call periodic:\n" + e);
            }
        }
    }

    @Test
    public void testAllSubsystemsCallPeriodicOnMotorControllers() throws Exception {
        // Find all subsystem implementations via reflection
        List<Object> subsystems = new ArrayList<>();
        for (var injectorMethod : getInjectorComponent().getClass().getMethods()) {
            if (injectorMethod.getParameterCount() != 0) {
                continue;
            }
            if (BaseSubsystem.class.isAssignableFrom(injectorMethod.getReturnType())) {
                injectorMethod.setAccessible(true);
                subsystems.add(injectorMethod.invoke(getInjectorComponent()));
            }
        }
        assertNotEquals(0, subsystems.size());

        // For each subsystem, find all the motor controllers
        for (Object subsystem : subsystems) {
            List<Object> motorControllers = new ArrayList<>();
            for (Field field : subsystem.getClass().getFields()) {
                if (XCANMotorController.class.isAssignableFrom(field.getType())) {
                    field.setAccessible(true);
                    field.set(subsystem, Mockito.spy(field.get(subsystem)));
                    // Replace the original motor controllers with the mocked ones
                    motorControllers.add(field.get(subsystem));
                }
            }

            if (motorControllers.isEmpty()) {
                continue;
            }

            // Run each subsystem refreshDataFrame method
            Method refreshDataFrameMethod = subsystem.getClass().getMethod("refreshDataFrame");
            refreshDataFrameMethod.setAccessible(true);
            try {
                refreshDataFrameMethod.invoke(subsystem);
            } catch (Exception e) {
                fail("Subsystem " + subsystem.getClass().getName() + " failed to call refreshDataFrame:\n" + e);
            }

            // Run each subsystem periodic method
            Method periodicMethod = subsystem.getClass().getMethod("periodic");
            periodicMethod.setAccessible(true);
            try {
                periodicMethod.invoke(subsystem);
            } catch (Exception e) {
                fail("Subsystem " + subsystem.getClass().getName() + " failed to call periodic:\n" + e);
            }

            // Check that each motor controller periodic method was called
            for (Object motorController : motorControllers) {
                    Mockito.verify((XCANMotorController)motorController, Mockito.times(1)
                            .description("Subsystem " + subsystem.getClass().getName() + " did not call periodic on a motor controller.")).periodic();
            }
        }
    }
}