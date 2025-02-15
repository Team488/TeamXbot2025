package competition.subsystems.coral_scorer.commands;

import competition.subsystems.coral_arm.CoralArmSubsystem;
import competition.subsystems.coral_scorer.CoralScorerSubsystem;
import competition.subsystems.elevator.ElevatorSubsystem;
import xbot.common.command.BaseCommand;

import javax.inject.Inject;

public class ScoreWhenReadyCommand extends BaseCommand {
    CoralScorerSubsystem coralScorerSubsystem;
    CoralArmSubsystem armPivotSubsystem;
    ElevatorSubsystem elevatorSubsystem;

    @Inject
    public ScoreWhenReadyCommand(CoralScorerSubsystem coralScorerSubsystem, CoralArmSubsystem armPivotSubsystem,
                                 ElevatorSubsystem elevatorSubsystem) {
        this.coralScorerSubsystem = coralScorerSubsystem;
        this.armPivotSubsystem = armPivotSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        addRequirements(coralScorerSubsystem, armPivotSubsystem, elevatorSubsystem);
    }

    @Override
    public void initialize() {
        log.info("Initializing");
    }

    @Override
    public void execute() {
        var hasCoral = coralScorerSubsystem.hasCoral();
        var getIsTargetAngleScoring = armPivotSubsystem.getIsTargetAngleScoring();
        var armMaintainer = armPivotSubsystem.isMaintainerAtGoal();
        var elevatorMaintainer = elevatorSubsystem.isMaintainerAtGoal();
        if (hasCoral && getIsTargetAngleScoring && armMaintainer && elevatorMaintainer) {
            coralScorerSubsystem.setCoralScorerState(CoralScorerSubsystem.CoralScorerState.SCORING);

        }
    }

    @Override
    public boolean isFinished() {
        return coralScorerSubsystem.confidentlyHasScoredCoral();
    }
}
