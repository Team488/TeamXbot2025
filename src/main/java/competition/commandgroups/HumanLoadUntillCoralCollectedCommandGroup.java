package competition.commandgroups;

import competition.subsystems.coral_scorer.commands.IntakeUntilCoralCollectedCommand;
import competition.subsystems.pose.Landmarks;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import xbot.common.command.BaseParallelCommandGroup;

import javax.inject.Inject;


public class HumanLoadUntillCoralCollectedCommandGroup extends BaseParallelCommandGroup {

    @Inject
    public HumanLoadUntillCoralCollectedCommandGroup(PrepCoralSystemCommandGroupFactory prepCoralSystemCommandGroupFactory,
                                                     IntakeUntilCoralCollectedCommand intakeUntilCoralCollectedCommand){

        var prepCoralSystemCommandGroup = prepCoralSystemCommandGroupFactory.create(() -> Landmarks.CoralLevel.COLLECTING);
        this.addCommands(intakeUntilCoralCollectedCommand, prepCoralSystemCommandGroup);
    }
}
