package competition.commandgroups.vision_path;

import competition.commandgroups.PrepCoralSystemCommandGroupFactory;
import competition.subsystems.coral_scorer.commands.IntakeUntilCoralCollectedCommand;
import competition.subsystems.drive.commands.vision_path.DriveVectorSmallCommand;
import competition.subsystems.drive.commands.vision_path.PathDriveToLocationCommand;
import competition.subsystems.pose.Landmarks;
import competition.subsystems.pose.vision.Paths;
import competition.subsystems.vision.CoprocessorCommunicationSubsystem;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.kobe.xbot.Utilities.Entities.XTableValues;
import xbot.common.command.NamedInstantCommand;

import javax.inject.Inject;
import javax.inject.Provider;

import static edu.wpi.first.units.Units.Degrees;

public class PathDriveToLocationAndIntakeUntilCollectedFactory {
    Provider<PathDriveToLocationCommand> pathDriveToLocationCommandProvider;
    PrepCoralSystemCommandGroupFactory prepCoralSystemCommandGroupFactory;
    Provider<IntakeUntilCoralCollectedCommand>
            intakeUntilCoralCollectedCommandProvider;
    Provider<DriveVectorSmallCommand> driveVectorSmallCommandProvider;
    CoprocessorCommunicationSubsystem coprocessorCommunicationSubsystem;

    @Inject
    public PathDriveToLocationAndIntakeUntilCollectedFactory(
            CoprocessorCommunicationSubsystem coprocessorCommunicationSubsystem,
            Provider<PathDriveToLocationCommand> pathDriveToLocationCommandProvider,
            PrepCoralSystemCommandGroupFactory prepCoralSystemCommandGroupFactory,
            Provider<IntakeUntilCoralCollectedCommand>
                    intakeUntilCoralCollectedCommandProvider,
            Provider<DriveVectorSmallCommand> driveVectorSmallCommand) {
        this.pathDriveToLocationCommandProvider =
                pathDriveToLocationCommandProvider;
        this.prepCoralSystemCommandGroupFactory =
                prepCoralSystemCommandGroupFactory;
        this.intakeUntilCoralCollectedCommandProvider =
                intakeUntilCoralCollectedCommandProvider;
        this.driveVectorSmallCommandProvider = driveVectorSmallCommand;
        this.coprocessorCommunicationSubsystem = coprocessorCommunicationSubsystem;
    }

    public ParallelDeadlineGroup create(
            XTableValues.BezierCurves override) {
        return create(null, override);
    }

    public ParallelDeadlineGroup create(
            Paths.Side side, XTableValues.BezierCurves override) {

        var driveToCoralStationSectionWhilePrepping = new ParallelCommandGroup();
        PathDriveToLocationCommand pathDriveToLocationCommand =
                pathDriveToLocationCommandProvider.get();

        DriveVectorSmallCommand driveVectorSmallCommand =
                driveVectorSmallCommandProvider.get();
        if (override != null) {
            if (side == null) {
                pathDriveToLocationCommand.setOverriddenPath(override);
            } else {
                pathDriveToLocationCommand.setOverriddenPath(side, override);
            }
        }
        SequentialCommandGroup driveToCoralStationThenDriveForward =
                new SequentialCommandGroup();
        if(override == null) {
            driveToCoralStationThenDriveForward.addCommands(new NamedInstantCommand("OverridePath", () -> {
                XTableValues.BezierCurves curves = coprocessorCommunicationSubsystem
                        .getLastCoralStationPath();
                if(curves != null) {
                    pathDriveToLocationCommand.setOverriddenPath(curves);
                    Angle finalDegrees = Degrees.of(curves.hasOptions()
                            ? curves.getOptions().hasFinalRotationDegrees()
                            ? curves.getOptions().getFinalRotationDegrees()
                            : 0
                            : 0);
                    pathDriveToLocationCommand.setOptions(
                            XTableValues.TraversalOptions.newBuilder()
                                    .setFinalRotationDegrees(finalDegrees.in(Degrees))
                                    .build());
                }
            }));
        }
        driveVectorSmallCommand.setBackwards(true);

        driveToCoralStationThenDriveForward.addCommands(pathDriveToLocationCommand,
                        driveVectorSmallCommand);
        var prepCoralSystem = prepCoralSystemCommandGroupFactory.create(
                () -> Landmarks.CoralLevel.COLLECTING);
        driveToCoralStationSectionWhilePrepping.addCommands(
                driveToCoralStationThenDriveForward, prepCoralSystem);
        IntakeUntilCoralCollectedCommand intakeUntilCoralCollectedCommand =
                intakeUntilCoralCollectedCommandProvider.get();
        return new ParallelDeadlineGroup(intakeUntilCoralCollectedCommand,
                driveToCoralStationSectionWhilePrepping);
    }
}
