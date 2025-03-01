package competition;

import competition.injection.components.SimulationComponent;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import org.apache.commons.lang3.reflect.FieldUtils;
import org.apache.commons.lang3.reflect.MethodUtils;
import org.junit.After;
import org.junit.Before;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.ReceiverThread;
import xbot.common.controls.sensors.XTimer;
import xbot.common.controls.sensors.XTimerImpl;
import xbot.common.math.PIDManager;
import xbot.common.properties.PropertyFactory;

import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.util.concurrent.TimeUnit;

import static org.junit.Assert.fail;

/**
 * Base class for all auto program tests that invoke the simulator.
 */
public class BaseSimulatedTest {
    private Thread robotThread;

    public PropertyFactory propertyFactory;

    protected PIDManager.PIDManagerFactory pf;

    protected XTimerImpl timer;
    protected Robot robot;

    protected SimulationComponent getInjectorComponent() {
        return (SimulationComponent)robot.getInjectorComponent();
    }

    @Before
    public void setUp() throws InterruptedException {
        SimHooks.restartTiming();

        // Reset AKit logger
        // Logger.reset();

        robot = new Robot();
        robotThread = new Thread(robot::startCompetition);
        robotThread.start();

        SimHooks.setProgramStarted();
        SimHooks.waitForProgramStart();

        if (!robot.reachedEndOfLoop.await(10, TimeUnit.SECONDS)) {
            fail("Robot did not start successfully");
        }

        timer = getInjectorComponent().timerImplementation();
        XTimer.setImplementation(timer);

        propertyFactory = getInjectorComponent().propertyFactory();
        pf = getInjectorComponent().pidFactory();
    }

    @After
    public void cleanup() throws InterruptedException {
        SimHooks.restartTiming();
        if (robot != null) {
            robot.endCompetition();
        }
        if (robotThread != null) {
            robotThread.join();
        }
        Logger.end();
    }
}
