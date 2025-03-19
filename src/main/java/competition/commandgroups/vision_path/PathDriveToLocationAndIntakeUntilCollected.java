package competition.commandgroups.vision_path;

import static edu.wpi.first.units.Units.Degrees;

import competition.commandgroups.PrepCoralSystemCommandGroupFactory;
import competition.subsystems.coral_scorer.commands.IntakeUntilCoralCollectedCommand;
import competition.subsystems.drive.commands.vision_path.DriveVectorSmallCommand;
import competition.subsystems.drive.commands.vision_path.PathDriveToLocationCommand;
import competition.subsystems.pose.Landmarks;
import competition.subsystems.pose.vision.Paths;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import javax.inject.Inject;
import javax.inject.Provider;
import org.kobe.xbot.Utilities.Entities.XTableValues;

public class PathDriveToLocationAndIntakeUntilCollected {
    Provider<PathDriveToLocationCommand> pathDriveToLocationCommandProvider;
    PrepCoralSystemCommandGroupFactory prepCoralSystemCommandGroupFactory;
    Provider<IntakeUntilCoralCollectedCommand>
            intakeUntilCoralCollectedCommandProvider;
    Provider<DriveVectorSmallCommand> driveVectorSmallCommandProvider;

    @Inject
    public PathDriveToLocationAndIntakeUntilCollected(
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
    }

    public ParallelDeadlineGroup create(
            Paths.Side side, XTableValues.BezierCurves override) {
        if (override == null) {
            return null;
        }
        var driveToCoralStationSectionWhilePrepping = new ParallelCommandGroup();
        PathDriveToLocationCommand pathDriveToLocationCommand =
                pathDriveToLocationCommandProvider.get();

        DriveVectorSmallCommand driveVectorSmallCommand =
                driveVectorSmallCommandProvider.get();
        pathDriveToLocationCommand.setOverriddenPath(side, override);
        Angle finalDegrees = Degrees.of(override.hasOptions()
                ? override.getOptions().hasFinalRotationDegrees()
                ? override.getOptions().getFinalRotationDegrees()
                : 0
                : 0);
        pathDriveToLocationCommand.setOptions(
                XTableValues.TraversalOptions.newBuilder()
                        .setFinalRotationDegrees(finalDegrees.in(Degrees))
                        .build());
        SequentialCommandGroup driveToCoralStationThenDriveForward =
                new SequentialCommandGroup(pathDriveToLocationCommand,
                        new InstantCommand(() -> {
                            driveVectorSmallCommand.setTargetAngle(
                                    DriverStation.getAlliance()
                                            .orElse(DriverStation.Alliance.Blue)
                                            .equals(DriverStation.Alliance.Red)
                                            ? finalDegrees
                                            : finalDegrees.plus(Degrees.of(180)));
                        }),
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
