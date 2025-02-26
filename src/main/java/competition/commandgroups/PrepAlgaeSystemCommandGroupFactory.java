package competition.commandgroups;

import competition.subsystems.algae_arm.AlgaeArmSubsystem;
import competition.subsystems.algae_arm.commands.SetAlgaeArmSetpointToTargetPosition;
import competition.subsystems.algae_collection.commands.AlgaeCollectionIntakeCommand;
import competition.subsystems.algae_collection.commands.AlgaeCollectionOutputCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import javax.inject.Inject;
import javax.inject.Provider;

public class PrepAlgaeSystemCommandGroupFactory extends ParallelCommandGroup {
    Provider<SetAlgaeArmSetpointToTargetPosition> setAlgaeArmSetpointToTargetPositionProvider;
    Provider<AlgaeCollectionOutputCommand> algaeCollectionOutputCommandProvider;
    Provider<AlgaeCollectionIntakeCommand> algaeCollectionIntakeCommandProvider;

    @Inject
    public PrepAlgaeSystemCommandGroupFactory(Provider<SetAlgaeArmSetpointToTargetPosition> setAlgaeArmSetpointToTargetPositionProvider,
                                              Provider<AlgaeCollectionOutputCommand> algaeCollectionOutputCommandProvider,
                                       Provider<AlgaeCollectionIntakeCommand> algaeCollectionIntakeCommandProvider) {
        this.setAlgaeArmSetpointToTargetPositionProvider = setAlgaeArmSetpointToTargetPositionProvider;
        this.algaeCollectionOutputCommandProvider = algaeCollectionOutputCommandProvider;
        this.algaeCollectionIntakeCommandProvider = algaeCollectionIntakeCommandProvider;
    }

    public ParallelCommandGroup create(AlgaeArmSubsystem.AlgaeArmPositions algaeArmPositions) {
        var group  = new ParallelCommandGroup();

        var setAlgaeArmSetpointToTarget = setAlgaeArmSetpointToTargetPositionProvider.get();
        setAlgaeArmSetpointToTarget.setTargetPosition(algaeArmPositions);

        var algaeCollectionOutput = algaeCollectionOutputCommandProvider.get();
        var algaeCollectionIntake = algaeCollectionIntakeCommandProvider.get();

        if (algaeArmPositions == AlgaeArmSubsystem.AlgaeArmPositions.GroundCollection) {
            group.addCommands(setAlgaeArmSetpointToTarget, algaeCollectionIntake);
        }
        else {
            group.addCommands(setAlgaeArmSetpointToTarget, algaeCollectionOutput);
        }
        return group;
    }
}
