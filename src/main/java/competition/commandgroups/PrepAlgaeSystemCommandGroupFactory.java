package competition.commandgroups;


import competition.subsystems.algae_collection.AlgaeCollectionSubsystem;
import competition.subsystems.algae_collection.commands.AlgaeCollectionIntakeCommand;
import competition.subsystems.algae_collection.commands.AlgaeCollectionOutputCommand;
import competition.subsystems.algae_collection.commands.AlgaeCollectionStopCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import javax.inject.Inject;
import javax.inject.Provider;

public class PrepAlgaeSystemCommandGroupFactory extends ParallelCommandGroup {
    Provider<AlgaeCollectionOutputCommand> algaeCollectionOutputCommandProvider;
    Provider<AlgaeCollectionIntakeCommand> algaeCollectionIntakeCommandProvider;
    Provider<AlgaeCollectionStopCommand> algaeCollectionStopCommandProvider;

    @Inject
    public PrepAlgaeSystemCommandGroupFactory(Provider<AlgaeCollectionOutputCommand> algaeCollectionOutputCommandProvider,
                                              Provider<AlgaeCollectionIntakeCommand> algaeCollectionIntakeCommandProvider,
                                              Provider<AlgaeCollectionStopCommand> algaeCollectionStopCommandProvider) {
        this.algaeCollectionOutputCommandProvider = algaeCollectionOutputCommandProvider;
        this.algaeCollectionIntakeCommandProvider = algaeCollectionIntakeCommandProvider;
        this.algaeCollectionStopCommandProvider = algaeCollectionStopCommandProvider;
    }

    public ParallelCommandGroup create(AlgaeCollectionSubsystem algaeCollectionSubsystem) {
        var group  = new ParallelCommandGroup();
        group.setName("PrepAlgaeSystemCommandGroup");


        var algaeCollectionOutput = algaeCollectionOutputCommandProvider.get();
        var algaeCollectionIntake = algaeCollectionIntakeCommandProvider.get();


        return group;
    }
}
