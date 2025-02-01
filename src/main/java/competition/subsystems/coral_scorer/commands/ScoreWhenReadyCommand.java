package competition.subsystems.coral_scorer.commands;

import competition.subsystems.coral_arm_pivot.CoralArmPivotSubsystem;
import competition.subsystems.coral_scorer.CoralScorerSubsystem;
import competition.subsystems.elevator.ElevatorSubsystem;
import xbot.common.command.BaseCommand;

import javax.inject.Inject;

public class ScoreWhenReadyCommand extends BaseCommand {
    CoralScorerSubsystem coralScorerSubsystem;
    CoralArmPivotSubsystem armPivotSubsystem;
    ElevatorSubsystem elevatorSubsystem;

    @Inject
    public ScoreWhenReadyCommand(CoralScorerSubsystem coralScorerSubsystem, CoralArmPivotSubsystem armPivotSubsystem,
                                 ElevatorSubsystem elevatorSubsystem) {
        this.coralScorerSubsystem = coralScorerSubsystem;
        this.armPivotSubsystem = armPivotSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        addRequirements(coralScorerSubsystem);
    }

    @Override
    public void initialize() {
        log.info("Initializing");
    }

    @Override
    public void execute() {
        if (coralScorerSubsystem.hasCoral() && armPivotSubsystem.getIsTargetAngleScoring()
                && armPivotSubsystem.isMaintainerAtGoal() && elevatorSubsystem.isMaintainerAtGoal()) {
            coralScorerSubsystem.scorer();
        }
    }

    @Override
    public boolean isFinished() {
        return coralScorerSubsystem.confidentlyHasScoredCoral();
    }
}
