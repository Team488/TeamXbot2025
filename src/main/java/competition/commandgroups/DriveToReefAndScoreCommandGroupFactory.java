package competition.commandgroups;

import competition.subsystems.coral_arm.CoralArmSubsystem;
import competition.subsystems.coral_scorer.commands.ScoreWhenReadyCommand;
import competition.subsystems.pose.Landmarks;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import javax.inject.Inject;
import javax.inject.Provider;

public class DriveToReefAndScoreCommandGroupFactory {
    DriveToReefThenAlignCommandGroupFactory driveToReefThenAlignCommandGroupFactory;
    PrepCoralSystemCommandGroupFactory prepCoralSystemFactory;
    Provider<ScoreWhenReadyCommand> scoreWhenReadyProvider;
}
