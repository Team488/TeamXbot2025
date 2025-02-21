package competition.commandgroups;

import competition.subsystems.algae_arm.AlgaeArmSubsystem;
import competition.subsystems.algae_arm.commands.SetAlgaeArmSetpointToTargetPosition;
import competition.subsystems.algae_collection.commands.AlgaeCollectionIntakeCommand;
import competition.subsystems.algae_collection.commands.AlgaeCollectionOutputCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import javax.inject.Inject;

public class PrepAlgaeSystemCommandGroupFactory extends ParallelCommandGroup {
    SetAlgaeArmSetpointToTargetPosition setAlgaeArmSetpointToTargetPosition;
    AlgaeCollectionOutputCommand algaeCollectionOutputCommand;
    AlgaeCollectionIntakeCommand algaeCollectionIntakeCommand;

    @Inject
    public PrepAlgaeSystemCommandGroupFactory(SetAlgaeArmSetpointToTargetPosition setAlgaeArmSetpointToTargetPosition,
                                       AlgaeCollectionOutputCommand algaeCollectionOutputCommand,
                                       AlgaeCollectionIntakeCommand algaeCollectionIntakeCommand) {
        this.setAlgaeArmSetpointToTargetPosition = setAlgaeArmSetpointToTargetPosition;
        this.algaeCollectionOutputCommand = algaeCollectionOutputCommand;
        this.algaeCollectionIntakeCommand = algaeCollectionIntakeCommand;
    }

    public ParallelCommandGroup create(AlgaeArmSubsystem.AlgaeArmPositions algaeArmPositions) {
        var group  = new ParallelCommandGroup();

        setAlgaeArmSetpointToTargetPosition.setTargetPosition(algaeArmPositions);
        if (algaeArmPositions == AlgaeArmSubsystem.AlgaeArmPositions.GroundCollection) {
            group.addCommands(setAlgaeArmSetpointToTargetPosition, algaeCollectionIntakeCommand);
        }
        else {
            group.addCommands(setAlgaeArmSetpointToTargetPosition, algaeCollectionOutputCommand);
        }
        return group;
    }
}
