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
        if (coralScorerSubsystem.hasCoral() && armPivotSubsystem.getIsTargetAngleScoring()
                && armPivotSubsystem.isMaintainerAtGoal() && elevatorSubsystem.isMaintainerAtGoal()) {
            coralScorerSubsystem.setCoralScorerState(CoralScorerSubsystem.CoralScorerState.SCORING);
            System.out.println("Has coral: " + coralScorerSubsystem.hasCoral());
            System.out.println("Arm at scoring angle: " + armPivotSubsystem.getIsTargetAngleScoring());
            System.out.println("Arm maintainer at goal: " + armPivotSubsystem.isMaintainerAtGoal());
            System.out.println("Elevator maintainer at goal: " + elevatorSubsystem.isMaintainerAtGoal());

        }
    }

    @Override
    public boolean isFinished() {
        return coralScorerSubsystem.confidentlyHasScoredCoral();
    }
}
