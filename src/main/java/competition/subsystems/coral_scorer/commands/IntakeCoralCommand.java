package competition.subsystems.coral_scorer.commands;

import competition.operator_interface.OperatorInterface;
import competition.subsystems.coral_scorer.CoralScorerSubsystem;
import xbot.common.command.BaseCommand;
import xbot.common.properties.DoubleProperty;
import xbot.common.properties.PropertyFactory;

import javax.inject.Inject;

public class IntakeCoralCommand extends BaseCommand {
    final CoralScorerSubsystem coralScorer;
    final OperatorInterface oi;
    final DoubleProperty rumbleIntensiveness;
    final DoubleProperty rumbleLength;

    @Inject
    IntakeCoralCommand(CoralScorerSubsystem coralScorerSubsystem, OperatorInterface oi, PropertyFactory propertyFactory) {
        this.coralScorer = coralScorerSubsystem;
        this.addRequirements(coralScorerSubsystem);
        this.oi = oi;
        propertyFactory.setPrefix(this);
        rumbleIntensiveness = propertyFactory.createPersistentProperty("Rumble Intensiveness", 1);
        rumbleLength = propertyFactory.createPersistentProperty("Rumble Length", 0.1);
    }

    @Override
    public void execute() {
        coralScorer.intake();

        if (coralScorer.hasCoral()) {
            oi.driverGamepad.getRumbleManager().rumbleGamepad(rumbleIntensiveness.get(), rumbleLength.get());
        }
    }
}
