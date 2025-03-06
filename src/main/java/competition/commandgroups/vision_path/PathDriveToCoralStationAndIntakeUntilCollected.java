package competition.commandgroups.vision_path;

import competition.commandgroups.PrepCoralSystemCommandGroupFactory;
import competition.subsystems.coral_scorer.commands.IntakeUntilCoralCollectedCommand;
import competition.subsystems.drive.commands.vision_path.DriveVectorSmallCommand;
import competition.subsystems.drive.commands.vision_path.PathDriveToNearestCoralStationSectionCommand;
import competition.subsystems.pose.Landmarks;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import javax.inject.Inject;

public class PathDriveToCoralStationAndIntakeUntilCollected {
    PathDriveToNearestCoralStationSectionCommand driveToCoralStationSectionCommand;
    PrepCoralSystemCommandGroupFactory prepCoralSystemCommandGroupFactory;
    IntakeUntilCoralCollectedCommand intakeUntilCoralCollectedCommand;
    DriveVectorSmallCommand driveVectorSmallCommand;


    @Inject
    public PathDriveToCoralStationAndIntakeUntilCollected(
            PathDriveToNearestCoralStationSectionCommand driveToCoralStationSectionCommand,
                                                          PrepCoralSystemCommandGroupFactory prepCoralSystemCommandGroupFactory,
                                                          IntakeUntilCoralCollectedCommand intakeUntilCoralCollectedCommand,
                                                          DriveVectorSmallCommand driveVectorSmallCommand) {
        this.driveToCoralStationSectionCommand = driveToCoralStationSectionCommand;
        this.prepCoralSystemCommandGroupFactory = prepCoralSystemCommandGroupFactory;
        this.intakeUntilCoralCollectedCommand = intakeUntilCoralCollectedCommand;
        this.driveVectorSmallCommand = driveVectorSmallCommand;
    }

    public ParallelDeadlineGroup create() {
        var driveToCoralStationSectionWhilePrepping = new ParallelCommandGroup();
        var driveToCoralStationThenDriveForward = new SequentialCommandGroup(
                driveToCoralStationSectionCommand,
                new InstantCommand(() -> {
                    driveVectorSmallCommand.setBackwards(true);
                    driveVectorSmallCommand.setTargetPose(driveToCoralStationSectionCommand.getTargetCoralStationSection());
                }),
                driveVectorSmallCommand
        );
        var prepCoralSystem = prepCoralSystemCommandGroupFactory.create(() -> Landmarks.CoralLevel.COLLECTING);
        driveToCoralStationSectionWhilePrepping.addCommands(driveToCoralStationThenDriveForward, prepCoralSystem);
        return new ParallelDeadlineGroup(intakeUntilCoralCollectedCommand, driveToCoralStationSectionWhilePrepping);

    }

}
