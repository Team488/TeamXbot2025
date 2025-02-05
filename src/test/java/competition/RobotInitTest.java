package competition;

import edu.wpi.first.wpilibj2.command.Subsystem;
import org.junit.Test;
import xbot.common.advantage.DataFrameRefreshable;
import xbot.common.command.BaseSubsystem;

import java.util.List;

public class RobotInitTest extends BaseCompetitionTest {
    @Test
    public void testDefaultSystem() {
        getInjectorComponent().subsystemDefaultCommandMap();
        getInjectorComponent().operatorCommandMap();
    }

    @Test
    public void robotDataflowTest() {
        List<Subsystem> subsystems = List.of(
                getInjectorComponent().driveSubsystem(),
                getInjectorComponent().coprocessorCommunicationSubsystem(),
                getInjectorComponent().aprilTagVisionSubsystemExtended(),
                getInjectorComponent().coprocessorCommunicationSubsystem(),
                getInjectorComponent().elevatorSubsystem(),
                getInjectorComponent().armPivotSubsystem(),
                getInjectorComponent().coralScorerSubsystem(),
                getInjectorComponent().superstructureMechanismSubsystem()
        );

        subsystems.forEach((a) -> {
            if (a instanceof DataFrameRefreshable) {
                ((DataFrameRefreshable) a).refreshDataFrame();
            }
        });
        // as above, but for the periodic call
        subsystems.forEach((a) -> {
            if (a instanceof BaseSubsystem) {
                ((BaseSubsystem) a).periodic();
            }
        });
    }
}