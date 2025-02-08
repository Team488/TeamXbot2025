package competition.commandgroups;

import competition.subsystems.coral_arm_pivot.commands.SetCoralArmTargetAngleCommand;
import competition.subsystems.coral_scorer.commands.IntakeUntillCoral;
import competition.subsystems.elevator.commands.SetElevatorTargetHeightCommand;
import competition.subsystems.pose.Landmarks;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import javax.inject.Inject;


public class HumanLoadUntillCoralCollectedCommandGroup extends ParallelCommandGroup {

    @Inject
    public HumanLoadUntillCoralCollectedCommandGroup(PrepCoralSystemCommandGroupFactory prepCoralSystemCommandGroupFactory,
                                                     IntakeUntillCoral intakeUntillCoral,
                                                     Landmarks.CoralLevel coralGoal){

        var prepCoralSystemCommandGroup = prepCoralSystemCommandGroupFactory.create(coralGoal);
        this.addCommands(intakeUntillCoral, prepCoralSystemCommandGroup);
    }
}
