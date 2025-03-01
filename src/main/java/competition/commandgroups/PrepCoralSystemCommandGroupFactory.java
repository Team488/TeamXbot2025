package competition.commandgroups;

import competition.subsystems.coral_arm.commands.SetCoralArmTargetAngleCommand;
import competition.subsystems.elevator.commands.SetElevatorTargetHeightCommand;
import competition.subsystems.pose.Landmarks;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import javax.inject.Inject;
import javax.inject.Provider;
import java.util.function.Supplier;

public class PrepCoralSystemCommandGroupFactory {

    Provider<SetElevatorTargetHeightCommand> setElevatorTargetHeightCommandProvider;
    Provider<SetCoralArmTargetAngleCommand> setCoralArmTargetAngleCommandProvider;


    @Inject
    public PrepCoralSystemCommandGroupFactory(Provider<SetElevatorTargetHeightCommand> setElevatorTargetHeightCommandProvider,
                                              Provider<SetCoralArmTargetAngleCommand> setCoralArmTargetAngleCommandProvider) {
        this.setElevatorTargetHeightCommandProvider = setElevatorTargetHeightCommandProvider;
        this.setCoralArmTargetAngleCommandProvider = setCoralArmTargetAngleCommandProvider;
    }

    public ParallelCommandGroup create(Supplier<Landmarks.CoralLevel> coralLevelSupplier) {
        var group = new ParallelCommandGroup();

        var setElevatorTargetHeightCommand = setElevatorTargetHeightCommandProvider.get();
        setElevatorTargetHeightCommand.setHeightSupplier(coralLevelSupplier);
        var setCoralArmTargetAngleCommand = setCoralArmTargetAngleCommandProvider.get();
        setCoralArmTargetAngleCommand.setAngleSupplier(coralLevelSupplier);

        group.addCommands(setElevatorTargetHeightCommand, setCoralArmTargetAngleCommand);

        return group;
    }


}
