package competition.subsystems.drive;

import competition.BaseSimulatedTest;
import competition.subsystems.pose.Landmarks;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.junit.runners.Parameterized;

import static org.hamcrest.Matchers.lessThan;

import static org.junit.Assert.assertThat;
import static org.junit.Assert.fail;

@RunWith(Parameterized.class)
public class AlignToTagGlobalMovementWithCalculatorTest extends BaseSimulatedTest {
    private final Pose2d startPose;
    private final DriverStation.Alliance alliance;
    private final double maxAllowedSeconds;

    @Parameterized.Parameters
    public static Object[][] data() {
        return new Object[][]{
                {Landmarks.BlueLeftStartingLine, DriverStation.Alliance.Blue, 14},
                {Landmarks.BlueLeftStartingLine, DriverStation.Alliance.Red, 14},
                {Landmarks.BlueRightStartingLine, DriverStation.Alliance.Blue, 14},
                {Landmarks.BlueLeftStartingLine, DriverStation.Alliance.Red, 14},
        };
    }

    public AlignToTagGlobalMovementWithCalculatorTest(Pose2d startPose, DriverStation.Alliance alliance, double maxAllowedSeconds) {
        this.startPose = startPose;
        this.alliance = alliance;
        this.maxAllowedSeconds = maxAllowedSeconds;
    }

    @Test
    public void testCommand() {
        var command = getInjectorComponent().alignToTagGlobalMovementCommand();
        command.setConfigurations(0, 21, false, 1);
        getInjectorComponent().autonomousCommandSelector().setCurrentAutonomousCommand(command);

        robot.getInjectorComponent().poseSubsystem().setCurrentPoseInMeters(startPose);

        DriverStationSim.setAllianceStationId(alliance == DriverStation.Alliance.Red
                ? AllianceStationID.Red1
                : AllianceStationID.Blue1);
        DriverStationSim.setDsAttached(true);
        DriverStationSim.setAutonomous(true);
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();

        var startTime = timer.getFPGATimestamp();

        while (!command.isFinished()) {
            if (robot.getScheduler().getNumberOfCrashes() > 3) {
                fail("Scheduler crashed too many times");
            }
            if (timer.getFPGATimestamp() - startTime > 15) {
                fail("Test timed out");
            }
            SimHooks.stepTiming(0.2);
        }
        SimHooks.pauseTiming();
        var endTime = timer.getFPGATimestamp();

        assertThat(endTime - startTime, lessThan(maxAllowedSeconds));
    }
}
