package competition.auto_programs.vision;

import competition.auto_programs.BaseAutonomousSequentialCommandGroup;
import competition.commandgroups.PrepCoralSystemCommandGroupFactory;
import competition.commandgroups.vision_path.PathDriveToCoralStationAndIntakeUntilCollected;
import competition.commandgroups.vision_path.PathToFaceAndScoreCommandGroupFactory;
import competition.simulation.BaseSimulator;
import competition.subsystems.pose.Landmarks;
import competition.subsystems.pose.PoseSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import xbot.common.subsystems.autonomous.AutonomousCommandSelector;

import javax.inject.Inject;
import javax.inject.Provider;

public class RightFourCoralAuto extends BaseAutonomousSequentialCommandGroup {

    @Inject
    public RightFourCoralAuto(AutonomousCommandSelector autoSelector,
                              PoseSubsystem pose,
                              Provider<PathToFaceAndScoreCommandGroupFactory> driveToFaceAndScoreFactProv,
                              Provider<PathDriveToCoralStationAndIntakeUntilCollected> driveToStationAndIntakeFactProv,
                              PrepCoralSystemCommandGroupFactory prepCoralSystemCommandGroupFact,
                              BaseSimulator simulator) {
        super(autoSelector);

        // Force our location to start in front of cage six
        var initializeStateCommand = pose.createSetPositionCommand(
                        () -> PoseSubsystem.convertBlueToRedIfNeeded(Landmarks.BlueCageSixStartingLine)
                )
                .alongWith(new InstantCommand(() -> simulator.resetPosition(PoseSubsystem.convertBlueToRedIfNeeded(Landmarks.BlueCageSixStartingLine))));
        this.addCommands(initializeStateCommand);

        // Drive to far Right, branch B and score level four
        var driveAndScoreFarRightBranchBLevelFour = driveToFaceAndScoreFactProv.get().create(
                        Landmarks.ReefFace.FAR_RIGHT, Landmarks.Branch.A, Landmarks.CoralLevel.FOUR)
                .alongWith(getDriveAndScoreStatusMessageCommand(Landmarks.ReefFace.FAR_RIGHT, Landmarks.Branch.A, Landmarks.CoralLevel.FOUR));
        this.addCommands(driveAndScoreFarRightBranchBLevelFour);

        // Drive to right coral station, far section and intake coral until collected
        var driveToRightStationAndIntakeFirst = driveToStationAndIntakeFactProv.get().create()
                .alongWith(
                        getDriveAndIntakeStatusMessageCommand(Landmarks.CoralStation.RIGHT, Landmarks.CoralStationSection.MID));
        this.addCommands(driveToRightStationAndIntakeFirst);

        // Drive to close right, branch B and score level four
        var driveAndScoreCloseRightBranchBLevelFour = driveToFaceAndScoreFactProv.get().create(
                        Landmarks.ReefFace.CLOSE_RIGHT, Landmarks.Branch.A, Landmarks.CoralLevel.FOUR)
                .alongWith(
                        getDriveAndScoreStatusMessageCommand(Landmarks.ReefFace.CLOSE_RIGHT, Landmarks.Branch.A, Landmarks.CoralLevel.FOUR));
        this.addCommands(driveAndScoreCloseRightBranchBLevelFour);

        // Drive to right coral station, close section and intake coral until collected
        var driveToRightStationAndIntakeSecond = driveToStationAndIntakeFactProv.get().create()
                .alongWith(
                        getDriveAndIntakeStatusMessageCommand(Landmarks.CoralStation.RIGHT, Landmarks.CoralStationSection.MID));
        this.addCommands(driveToRightStationAndIntakeSecond);

        // Drive to close right, branch A and score level four
        var driveAndScoreCloseRightBranchALevelFour = driveToFaceAndScoreFactProv.get().create(
                        Landmarks.ReefFace.CLOSE_RIGHT, Landmarks.Branch.B, Landmarks.CoralLevel.FOUR)
                .alongWith(
                        getDriveAndScoreStatusMessageCommand(Landmarks.ReefFace.CLOSE_RIGHT, Landmarks.Branch.B, Landmarks.CoralLevel.FOUR));
        this.addCommands(driveAndScoreCloseRightBranchALevelFour);

        // Drive to right coral station, far section and intake coral until collected again
        var driveToRightStationAndIntakeThird = driveToStationAndIntakeFactProv.get().create()
                .alongWith(
                        getDriveAndIntakeStatusMessageCommand(Landmarks.CoralStation.RIGHT, Landmarks.CoralStationSection.MID));
        this.addCommands(driveToRightStationAndIntakeThird);

        // Drive to close, branch A and score level four
        var driveAndScoreCloseBranchALevelFour = driveToFaceAndScoreFactProv.get().create(
                        Landmarks.ReefFace.CLOSE, Landmarks.Branch.B, Landmarks.CoralLevel.FOUR)
                .alongWith(
                        getDriveAndScoreStatusMessageCommand(Landmarks.ReefFace.CLOSE, Landmarks.Branch.B, Landmarks.CoralLevel.FOUR));
        this.addCommands(driveAndScoreCloseBranchALevelFour);

        var driveToRightStationAndIntakeFinal = driveToStationAndIntakeFactProv.get().create()
                .alongWith(
                        getDriveAndIntakeStatusMessageCommand(Landmarks.CoralStation.RIGHT, Landmarks.CoralStationSection.MID));
        this.addCommands(driveToRightStationAndIntakeFinal);
    }
}
