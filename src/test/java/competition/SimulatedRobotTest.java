package competition;

import competition.electrical_contract.ElectricalContract;
import competition.electrical_contract.UnitTestContract2025;
import org.junit.Test;
import xbot.common.command.BaseRobot;

import java.util.concurrent.TimeUnit;

import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

public class SimulatedRobotTest {
    @Test
    public void testSimulatedRobot() {
        try (var robot = new Robot()) {
            robot.simulatorContract = new UnitTestContract2025();

            Thread robotThread = new Thread(() -> {
                BaseRobot.startRobot(() -> robot);
            });
            robotThread.start();

            try {
                var passDisabledInit = robot.reachedDisabledInit.tryAcquire(10, TimeUnit.SECONDS);
                assertTrue("Robot did not reach disabled init", passDisabledInit);

                var passDisabledPeriodic = robot.reachedDisabledPeriodic.tryAcquire(10, TimeUnit.SECONDS);
                assertTrue("Robot did not reach disabled periodic", passDisabledPeriodic);
            } catch (InterruptedException e) {
                fail("Robot test was interrupted");
            } finally {
                robot.endCompetition();
            }
        }
    }
}
