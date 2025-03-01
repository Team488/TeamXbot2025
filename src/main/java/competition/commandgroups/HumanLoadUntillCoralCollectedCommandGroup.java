package competition.commandgroups;

import competition.subsystems.coral_scorer.commands.IntakeUntilCoralCollectedCommand;
import competition.subsystems.pose.Landmarks;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import javax.inject.Inject;


public class HumanLoadUntillCoralCollectedCommandGroup extends ParallelCommandGroup {

    @Inject
    public HumanLoadUntillCoralCollectedCommandGroup(PrepCoralSystemCommandGroupFactory prepCoralSystemCommandGroupFactory,
                                                     IntakeUntilCoralCollectedCommand intakeUntilCoralCollectedCommand){

        var prepCoralSystemCommandGroup = prepCoralSystemCommandGroupFactory.create(() -> Landmarks.CoralLevel.COLLECTING);
        this.addCommands(intakeUntilCoralCollectedCommand, prepCoralSystemCommandGroup);
    }
}
