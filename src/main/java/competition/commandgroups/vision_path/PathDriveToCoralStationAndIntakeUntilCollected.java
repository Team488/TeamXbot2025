package competition.commandgroups.vision_path;

import competition.commandgroups.PrepCoralSystemCommandGroupFactory;
import competition.subsystems.coral_scorer.commands.IntakeUntilCoralCollectedCommand;
import competition.subsystems.drive.commands.vision_path.DriveVectorSmallCommand;
import competition.subsystems.drive.commands.vision_path.PathDriveToNearestCoralStationSectionCommandCommand;
import competition.subsystems.pose.Landmarks;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import javax.inject.Inject;
import javax.inject.Provider;

public class PathDriveToCoralStationAndIntakeUntilCollected {
    Provider<PathDriveToNearestCoralStationSectionCommandCommand> driveToCoralStationSectionCommandProvider;
    PrepCoralSystemCommandGroupFactory prepCoralSystemCommandGroupFactory;
    Provider<IntakeUntilCoralCollectedCommand> intakeUntilCoralCollectedCommandProvider;
    Provider<DriveVectorSmallCommand> driveVectorSmallCommandProvider;


    @Inject
    public PathDriveToCoralStationAndIntakeUntilCollected(
            Provider<PathDriveToNearestCoralStationSectionCommandCommand> driveToCoralStationSectionCommandProvider,
            PrepCoralSystemCommandGroupFactory prepCoralSystemCommandGroupFactory,
            Provider<IntakeUntilCoralCollectedCommand> intakeUntilCoralCollectedCommandProvider,
            Provider<DriveVectorSmallCommand> driveVectorSmallCommand) {
        this.driveToCoralStationSectionCommandProvider = driveToCoralStationSectionCommandProvider;
        this.prepCoralSystemCommandGroupFactory = prepCoralSystemCommandGroupFactory;
        this.intakeUntilCoralCollectedCommandProvider = intakeUntilCoralCollectedCommandProvider;
        this.driveVectorSmallCommandProvider = driveVectorSmallCommand;
    }

    public ParallelDeadlineGroup create() {
        var driveToCoralStationSectionWhilePrepping = new ParallelCommandGroup();
        PathDriveToNearestCoralStationSectionCommandCommand pathDriveToNearestCoralStationSectionCommand = driveToCoralStationSectionCommandProvider.get();
        DriveVectorSmallCommand driveVectorSmallCommand = driveVectorSmallCommandProvider.get();

        var driveToCoralStationThenDriveForward = new SequentialCommandGroup(
                pathDriveToNearestCoralStationSectionCommand,
                new InstantCommand(() -> {
                    driveVectorSmallCommand.setBackwards(true);
                    driveVectorSmallCommand.setTargetPose(pathDriveToNearestCoralStationSectionCommand.getTargetCoralStationSection());
                }),
                driveVectorSmallCommand
        );
        var prepCoralSystem = prepCoralSystemCommandGroupFactory.create(() -> Landmarks.CoralLevel.COLLECTING);
        driveToCoralStationSectionWhilePrepping.addCommands(driveToCoralStationThenDriveForward, prepCoralSystem);
        IntakeUntilCoralCollectedCommand intakeUntilCoralCollectedCommand = intakeUntilCoralCollectedCommandProvider.get();
        return new ParallelDeadlineGroup(intakeUntilCoralCollectedCommand, driveToCoralStationSectionWhilePrepping);

    }

}
