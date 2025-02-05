package competition.commandgroups;

import competition.subsystems.coral_arm_pivot.CoralArmPivotSubsystem;
import competition.subsystems.coral_arm_pivot.commands.SetCoralArmTargetAngleCommand;
import competition.subsystems.elevator.ElevatorSubsystem;
import competition.subsystems.elevator.commands.SetElevatorTargetHeightCommand;
import competition.subsystems.pose.Landmarks;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import javax.inject.Inject;
import javax.inject.Provider;

public class PrepCoralSystemCommandGroupFactory {

    Provider<SetElevatorTargetHeightCommand> setElevatorTargetHeightCommandProvider;
    Provider<SetCoralArmTargetAngleCommand> setCoralArmTargetAngleCommandProvider;


    @Inject
    public PrepCoralSystemCommandGroupFactory(Provider<SetElevatorTargetHeightCommand> setElevatorTargetHeightCommandProvider,
                                              Provider<SetCoralArmTargetAngleCommand> setCoralArmTargetAngleCommandProvider) {
        this.setElevatorTargetHeightCommandProvider = setElevatorTargetHeightCommandProvider;
        this.setCoralArmTargetAngleCommandProvider = setCoralArmTargetAngleCommandProvider;
    }

    public ParallelCommandGroup create(Landmarks.CoralLevel coralGoal) {
        var group = new ParallelCommandGroup();

        var setElevatorTargetHeightCommand = setElevatorTargetHeightCommandProvider.get();
        setElevatorTargetHeightCommand.setHeight(coralGoal);
        var setCoralArmTargetAngleCommand = setCoralArmTargetAngleCommandProvider.get();
        setCoralArmTargetAngleCommand.setAngle(coralGoal);

        group.addCommands(setElevatorTargetHeightCommand, setCoralArmTargetAngleCommand);

        return group;
    }


}
