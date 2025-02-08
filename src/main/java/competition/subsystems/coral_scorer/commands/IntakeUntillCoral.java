package competition.subsystems.coral_scorer.commands;

import competition.subsystems.coral_scorer.CoralScorerSubsystem;
import xbot.common.command.BaseCommand;

import javax.inject.Inject;

public class IntakeUntillCoral extends BaseCommand {
    CoralScorerSubsystem coral;

    @Inject
    public IntakeUntillCoral (CoralScorerSubsystem coralScorerSubsystem){
        coral= coralScorerSubsystem;
        this.addRequirements(coral);
    }
    @Override
    public void initialize(){}

    @Override
    public boolean isFinished() {
        if (this.coral.confidentlyHasCoral()) {
            coral.motor.setPower(0);
            return true;
        } else {
            coral.motor.setPower(coral.intakePower.get());
            return false;
        }
    }
}
