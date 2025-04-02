package competition.subsystems.stinger_collection.commands;

import competition.operator_interface.OperatorInterface;
import competition.subsystems.stinger_collection.StingerCollectionSubsystem;
import xbot.common.command.BaseCommand;

import javax.inject.Inject;

public class IntakeCoralCommand extends BaseCommand {
    final StingerCollectionSubsystem coral;
    final OperatorInterface oi;

    @Inject
    public IntakeCoralCommand(StingerCollectionSubsystem coralScorerSubsystem, OperatorInterface oi) {
        coral = coralScorerSubsystem;
        this.oi = oi;
        this.addRequirements(coral);
    }

    @Override
    public void initialize() {
        coral.setStingerCollectionState(StingerCollectionSubsystem.StingerCollectionState.INTAKING_CORAL);
    }


    @Override
    public void execute(){
        if (coral.confidentlyHasCoral()) {
            oi.operatorGamepad.getRumbleManager().rumbleGamepad(oi.getOperatorGamepadRumbleIntensitiy(),.1);
            oi.driverGamepad.getRumbleManager().rumbleGamepad(oi.getDriverGamepadRumbleIntensitity(), .1);
        }
    }

}
