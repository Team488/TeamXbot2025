package competition.subsystems.drive.commands;

import competition.commandgroups.PrepCoralSystemCommandGroupFactory;
import competition.subsystems.coral_scorer.commands.IntakeUntilCoralCollectedCommand;
import competition.subsystems.pose.Landmarks;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;

import javax.inject.Inject;

public class AlignToHLSAndIntakeUntilCollectedCommandGroupFactory {
    AlignToSpecificHumanLoadingStationCommand alignToSpecificHumanLoadingStationCommand;
    PrepCoralSystemCommandGroupFactory prepCoralSystemCommandGroupFactory;
    IntakeUntilCoralCollectedCommand intakeUntilCoralCollectedCommand;

    @Inject
    public AlignToHLSAndIntakeUntilCollectedCommandGroupFactory(AlignToSpecificHumanLoadingStationCommand
                                                                            alignToSpecificHumanLoadingStationCommand,
                                                                PrepCoralSystemCommandGroupFactory
                                                                        prepCoralSystemCommandGroupFactory,
                                                                IntakeUntilCoralCollectedCommand
                                                                            intakeUntilCoralCollectedCommand) {
        this.alignToSpecificHumanLoadingStationCommand = alignToSpecificHumanLoadingStationCommand;
        this.prepCoralSystemCommandGroupFactory = prepCoralSystemCommandGroupFactory;
        this.intakeUntilCoralCollectedCommand = intakeUntilCoralCollectedCommand;
    }

    public ParallelDeadlineGroup create(Landmarks.CoralStation station) {
        var driveToCoralStationSectionWhilePrepping = new ParallelCommandGroup();
        var prepCoralSystem = prepCoralSystemCommandGroupFactory.create(() -> Landmarks.CoralLevel.COLLECTING);
        alignToSpecificHumanLoadingStationCommand.setCoralStation(station);
        driveToCoralStationSectionWhilePrepping.addCommands(alignToSpecificHumanLoadingStationCommand, prepCoralSystem);

        return new ParallelDeadlineGroup(intakeUntilCoralCollectedCommand, driveToCoralStationSectionWhilePrepping);

    }

}
