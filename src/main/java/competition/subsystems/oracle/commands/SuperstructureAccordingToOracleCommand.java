package competition.subsystems.oracle.commands;

import competition.subsystems.coral_arm_pivot.CoralArmPivotSubsystem;
import competition.subsystems.coral_scorer.CoralScorerSubsystem;
import competition.subsystems.elevator.ElevatorSubsystem;
import competition.subsystems.oracle.OracleSubsystem;
import competition.subsystems.oracle.OracleSuperstructureAdvice;
import xbot.common.command.BaseCommand;

import javax.inject.Inject;

public class SuperstructureAccordingToOracleCommand extends BaseCommand {

    final ElevatorSubsystem elevator;
    final CoralArmPivotSubsystem coralArm;
    final CoralScorerSubsystem coralScorer;
    final OracleSubsystem oracle;

    OracleSuperstructureAdvice advice;

    @Inject
    public SuperstructureAccordingToOracleCommand(OracleSubsystem oracle, ElevatorSubsystem elevator, CoralArmPivotSubsystem coralArm, CoralScorerSubsystem coralScorer) {
        this.oracle = oracle;
        this.elevator = elevator;
        this.coralArm = coralArm;
        this.coralScorer = coralScorer;

        addRequirements(elevator.getSetpointLock(), coralArm.getSetpointLock(), coralScorer);
    }

    public void initialize() {
        log.info("Initializing");
        oracle.requestReevaluation();
        setNewInstruction();
    }

    private void setNewInstruction() {
        this.advice = oracle.getSuperstructureAdvice();
    }

    @Override
    public void execute() {
        if (oracle.getSuperstructureAdvice().instructionNumber != advice.instructionNumber) {
            setNewInstruction();
        }

        elevator.setTargetHeight(advice.coralLevelToAchieve);
        coralArm.setTargetAngle(advice.coralLevelToAchieve);
        coralScorer.setPowerForState(advice.desiredScorerState);
    }
}
