package competition.subsystems.oracle.commands;

import competition.subsystems.oracle.CoralStationMode;
import competition.subsystems.oracle.OracleSubsystem;
import xbot.common.command.BaseCommand;
import xbot.common.logging.RobotAssertionManager;

import javax.inject.Inject;

// Maybe we should make a toggle command as well...
public class SetAllowedOracleCoralStationCommand extends BaseCommand {

    private final OracleSubsystem oracle;
    private final RobotAssertionManager assertionManager;
    private CoralStationMode modeToSet = null;

    public void setCoralStationMode(CoralStationMode mode) {
        // Allow only setting the mode once
        if (modeToSet != null) {
            assertionManager.throwException("Set CoralStationMode more than once for toggle command!", new Exception());
            return;
        }
        modeToSet = mode;
    }

    @Inject
    public SetAllowedOracleCoralStationCommand(OracleSubsystem oracle, RobotAssertionManager assertionManager) {
        this.oracle = oracle;
        this.assertionManager = assertionManager;
    }

    @Override
    public void initialize() {
        if (modeToSet == null) {
            assertionManager.throwException("Mode to toggle not set!", new Exception());
            return;
        }
        oracle.setCoralStationMode(modeToSet);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
