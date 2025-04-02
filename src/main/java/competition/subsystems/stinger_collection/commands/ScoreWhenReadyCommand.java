package competition.subsystems.stinger_collection.commands;

import competition.subsystems.stinger_arm.StingerArmSubsystem;
import competition.subsystems.stinger_collection.StingerCollectionSubsystem;
import competition.subsystems.elevator.ElevatorSubsystem;
import xbot.common.command.BaseCommand;

import javax.inject.Inject;

public class ScoreWhenReadyCommand extends BaseCommand {
    StingerCollectionSubsystem coralScorerSubsystem;
    StingerArmSubsystem coralArmSubsystem;
    ElevatorSubsystem elevatorSubsystem;

    @Inject
    public ScoreWhenReadyCommand(StingerCollectionSubsystem coralScorerSubsystem, StingerArmSubsystem coralArmSubsystem,
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
            coralScorerSubsystem.setStingerCollectionState(StingerCollectionSubsystem.StingerCollectionState.SCORING_CORAL);
        }
        aKitLog.record("isTargetAngleScoring", getIsTargetAngleScoring);
        aKitLog.record("armPrepped", armMaintainerAtGoal);
        aKitLog.record("elevatorPrepped", elevatorMaintainerAtGoal);
    }

    @Override
    public boolean isFinished() {
        return coralScorerSubsystem.confidentlyHasScoredCoral();
    }
}
