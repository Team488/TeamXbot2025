package competition.subsystems.oracle.commands;

import competition.subsystems.oracle.OracleSubsystem;
import competition.subsystems.oracle.ScoringTask;
import xbot.common.command.BaseCommand;
import xbot.common.logging.RobotAssertionManager;

import javax.inject.Inject;

public class AddToOracleScoringQueueCommand extends BaseCommand {

    private final RobotAssertionManager assertionManager;
    private final OracleSubsystem oracle;
    private ScoringTask taskToAdd;

    public void setTaskToAdd(ScoringTask taskToAdd) {
        this.taskToAdd = taskToAdd;
    }

    @Inject
    public AddToOracleScoringQueueCommand(OracleSubsystem oracle, RobotAssertionManager assertionManager) {
        this.assertionManager = assertionManager;
        this.oracle = oracle;
    }

    @Override
    public void initialize() {
        if (taskToAdd == null) {
            assertionManager.throwException("Command used with ScoringTask not initialized", new Exception());
        }
        oracle.addScoringTask(taskToAdd);
    }

    @Override
    public boolean isFinished() {return true;}
}
