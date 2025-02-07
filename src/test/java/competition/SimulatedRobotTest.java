package competition;

import competition.electrical_contract.UnitTestContract2025;
import edu.wpi.first.wpilibj.RobotBase;
import org.junit.Test;

import java.util.concurrent.TimeUnit;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

public class SimulatedRobotTest {
    @Test
    public void testSimulatedRobot() {
        RobotBase.suppressExitWarning(true);
        try (var robot = new Robot()) {
            robot.simulatorContract = new UnitTestContract2025();

            Thread robotThread = new Thread(robot::startCompetition);
            robotThread.start();

            try {
                var passDisabledInit = robot.reachedDisabledInit.tryAcquire(5, TimeUnit.SECONDS);
                assertTrue("Robot did not reach disabled init", passDisabledInit);

                var passDisabledPeriodic = robot.reachedEndOfLoop.tryAcquire(5, TimeUnit.SECONDS);
                assertTrue("Robot did not reach disabled periodic", passDisabledPeriodic);

                assertEquals(0, robot.getScheduler().getNumberOfCrashes());
            } catch (InterruptedException e) {
                fail("Robot test was interrupted");
            } finally {
                robot.endCompetition();

                try {
                    robotThread.join(1000);
                } catch (InterruptedException e) {
                    fail("Thread was interrupted");
                }
            }
        }
    }
}
