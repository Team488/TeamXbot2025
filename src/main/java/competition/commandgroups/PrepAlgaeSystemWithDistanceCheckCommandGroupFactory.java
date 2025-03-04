package competition.commandgroups;

import competition.subsystems.algae_arm.AlgaeArmSubsystem;
import competition.subsystems.drive.commands.MeasureDistanceToFaceCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import javax.inject.Inject;

import static edu.wpi.first.units.Units.Meters;

public class PrepAlgaeSystemWithDistanceCheckCommandGroupFactory {

    PrepAlgaeSystemCommandGroupFactory prepAlgaeSystemCommandGroupFact;
    MeasureDistanceToFaceCommand measureDistanceToFaceCommand;

    @Inject
    public PrepAlgaeSystemWithDistanceCheckCommandGroupFactory(PrepAlgaeSystemCommandGroupFactory prepAlgaeSystemCommandGroupFact,
                                                               MeasureDistanceToFaceCommand measureDistanceToFaceCommand) {
        this.prepAlgaeSystemCommandGroupFact = prepAlgaeSystemCommandGroupFact;
        this.measureDistanceToFaceCommand = measureDistanceToFaceCommand;
    }

    public SequentialCommandGroup create(AlgaeArmSubsystem.AlgaeArmPositions algaeArmPosition,
                                         double thresholdInMeters) {
        var prepIfOutsideThreshold = new SequentialCommandGroup();

        measureDistanceToFaceCommand.setDistanceThreshold(Meters.of(thresholdInMeters));
        var prepAlgae = prepAlgaeSystemCommandGroupFact.create(algaeArmPosition);

        prepIfOutsideThreshold.addCommands(measureDistanceToFaceCommand, prepAlgae);

        return prepIfOutsideThreshold;
    }
}
