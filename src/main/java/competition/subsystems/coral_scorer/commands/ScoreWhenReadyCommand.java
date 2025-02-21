package competition.subsystems.coral_scorer.commands;

import competition.subsystems.coral_arm.CoralArmSubsystem;
import competition.subsystems.coral_scorer.CoralScorerSubsystem;
import competition.subsystems.elevator.ElevatorSubsystem;
import xbot.common.command.BaseCommand;

import javax.inject.Inject;

public class ScoreWhenReadyCommand extends BaseCommand {
    CoralScorerSubsystem coralScorerSubsystem;
    CoralArmSubsystem coralArmSubsystem;
    ElevatorSubsystem elevatorSubsystem;

    @Inject
    public ScoreWhenReadyCommand(CoralScorerSubsystem coralScorerSubsystem, CoralArmSubsystem coralArmSubsystem,
                                 ElevatorSubsystem elevatorSubsystem) {
        this.coralScorerSubsystem = coralScorerSubsystem;
        this.coralArmSubsystem = coralArmSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        addRequirements(coralScorerSubsystem);
    }

    @Override
    public void initialize() {
        log.info("Initializing");
    }

    @Override
    public void execute() {
        var hasCoral = coralScorerSubsystem.hasCoral();
        var getIsTargetAngleScoring = coralArmSubsystem.getIsTargetAngleScoring();
        var armMaintainerAtGoal = coralArmSubsystem.isMaintainerAtGoal();
        var elevatorMaintainerAtGoal = elevatorSubsystem.isMaintainerAtGoal();
        if (hasCoral && getIsTargetAngleScoring && armMaintainerAtGoal && elevatorMaintainerAtGoal) {
            coralScorerSubsystem.setCoralScorerState(CoralScorerSubsystem.CoralScorerState.SCORING);
        }

    }

    @Override
    public boolean isFinished() {
        return coralScorerSubsystem.confidentlyHasScoredCoral();
    }
}
