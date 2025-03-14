package competition.subsystems.drive.commands.vision_path;

import competition.commandgroups.PrepCoralSystemCommandGroupFactory;
import competition.subsystems.coral_scorer.commands.IntakeUntilCoralCollectedCommand;
import competition.subsystems.drive.commands.DriveToCoralStationInterstitialCommand;
import competition.subsystems.drive.commands.ShoveCoralStationCommand;
import competition.subsystems.pose.Landmarks;
import competition.subsystems.pose.PoseSubsystem;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.kobe.xbot.Utilities.Entities.XTableValues;

import javax.inject.Inject;
import javax.inject.Provider;

public class PathToStationAndIntakeUntilCollectedCommandGroup extends ParallelDeadlineGroup {
    PoseSubsystem pose;
    Landmarks.CoralStation station;
    private XTableValues.BezierCurves curves;

    @Inject
    public PathToStationAndIntakeUntilCollectedCommandGroup(PrepCoralSystemCommandGroupFactory prepCoralSystemCommandGroupFactory,
                                                            IntakeUntilCoralCollectedCommand intakeUntilCoralCollectedCommand,
                                                            ShoveCoralStationCommand shoveCoralStationCommand,
                                                            PathToNearestCoralStationSectionCommand pathToNearestCoralStationCommand,
                                                            PoseSubsystem pose) {
        super(intakeUntilCoralCollectedCommand);
        this.pose = pose;
        this.curves = pathToNearestCoralStationCommand.getCurves();

        // Prep coral system to coral collection
        var prepCoralSystem = prepCoralSystemCommandGroupFactory.create(() -> Landmarks.CoralLevel.COLLECTING);
        this.addCommands(prepCoralSystem);

        shoveCoralStationCommand.setShoveAngle(station);
        this.addCommands(pathToNearestCoralStationCommand.andThen(shoveCoralStationCommand.withTimeout(4)));
    }

    public void setStation(Landmarks.CoralStation station) {
        this.station = station;
    }

    public XTableValues.BezierCurves getCurves() {
        return curves;
    }
}
