package competition.auto_programs.vision;

import static competition.subsystems.pose.vision.Paths.blueCloseBranchAToLeftCoralStation;
import static competition.subsystems.pose.vision.Paths.blueCloseLeftBranchAToLeftCoralStation;
import static competition.subsystems.pose.vision.Paths.blueCloseLeftBranchBToLeftCoralStation;
import static competition.subsystems.pose.vision.Paths.farLeftBToCoralStationBlue;

import competition.auto_programs.BaseAutonomousSequentialCommandGroup;
import competition.commandgroups.DriveToFaceAndScoreCommandGroupFactory;
import competition.commandgroups.PrepCoralSystemCommandGroupFactory;
import competition.commandgroups.vision_path.PathDriveToCoralStationAndIntakeUntilCollected;
import competition.commandgroups.vision_path.PathDriveToLocationForCoralStationAndIntakeUntilCollectedFactory;
import competition.commandgroups.vision_path.PathToFaceAndScoreCommandGroupFactory;
import competition.electrical_contract.ElectricalContract;
import competition.simulation.BaseSimulator;
import competition.subsystems.pose.Landmarks;
import competition.subsystems.pose.PoseSubsystem;
import competition.subsystems.pose.vision.Paths;
import competition.subsystems.vision.CoprocessorCommunicationSubsystem;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import javax.inject.Inject;
import javax.inject.Provider;
import xbot.common.subsystems.autonomous.AutonomousCommandSelector;

public class RightFourCoralAuto extends BaseAutonomousSequentialCommandGroup {
    @Inject
    public RightFourCoralAuto(AprilTagFieldLayout aprilTagFieldLayout,
                              ElectricalContract electricalContract,
                              AutonomousCommandSelector autoSelector,
                              CoprocessorCommunicationSubsystem coprocessorCommunicationSubsystem,
                              PoseSubsystem pose,
                              Provider<PathDriveToCoralStationAndIntakeUntilCollected>
                                      driveToStationAndIntakeFactProv,
                              DriveToFaceAndScoreCommandGroupFactory driveToFaceAndScoreFact,
                              Provider<PathToFaceAndScoreCommandGroupFactory>
                                      driveToFaceAndScoreFactProv,
                              Provider<PathDriveToLocationForCoralStationAndIntakeUntilCollectedFactory>
                                      pathDriveToLocationAndIntakeUntilCollectedProvider,
                              PrepCoralSystemCommandGroupFactory prepCoralSystemCommandGroupFact,
                              BaseSimulator simulator) {
        super(autoSelector);
        var initializeStateCommand =
                pose.createSetPositionCommand(
                                ()
                                        -> PoseSubsystem.convertBlueToRedIfNeeded(
                                        Landmarks.BlueCageSixStartingLine))
                        .alongWith(new InstantCommand(
                                ()
                                        -> simulator.resetPosition(
                                        PoseSubsystem.convertBlueToRedIfNeeded(
                                                Landmarks.BlueCageSixStartingLine))));
        this.addCommands(initializeStateCommand);
        // Score 1
        getDriveAndScoreStatusMessageCommand(Landmarks.ReefFace.FAR_RIGHT,
                Landmarks.Branch.A, Landmarks.CoralLevel.FOUR);
        var driveAndScoreFarLeftBranchBLevelFour =
                driveToFaceAndScoreFact
                        .create(Landmarks.ReefFace.FAR_RIGHT, Landmarks.Branch.A,
                                Landmarks.CoralLevel.FOUR)
                        .alongWith(getDriveAndScoreStatusMessageCommand(
                                Landmarks.ReefFace.FAR_RIGHT, Landmarks.Branch.A,
                                Landmarks.CoralLevel.FOUR));
        this.addCommands(driveAndScoreFarLeftBranchBLevelFour);

        // Coral Station
        var driveToLeftStation =
                pathDriveToLocationAndIntakeUntilCollectedProvider.get()
                        .create(Paths.Side.RIGHT, farLeftBToCoralStationBlue)
                        .alongWith(getDriveAndIntakeStatusMessageCommand(
                                Landmarks.CoralStation.RIGHT,
                                Landmarks.CoralStationSection.MID));

        this.addCommands(driveToLeftStation);

        // Score 2
        var driveAndScoreCloseLeftBranchBLevelFour =
                driveToFaceAndScoreFact
                        .create(Landmarks.ReefFace.CLOSE_RIGHT, Landmarks.Branch.A,
                                Landmarks.CoralLevel.FOUR)
                        .alongWith(getDriveAndScoreStatusMessageCommand(
                                Landmarks.ReefFace.CLOSE_RIGHT, Landmarks.Branch.A,
                                Landmarks.CoralLevel.FOUR));
        this.addCommands(driveAndScoreCloseLeftBranchBLevelFour);

        // Coral Station
        var driveToRightStationAndIntakeSecond =
                pathDriveToLocationAndIntakeUntilCollectedProvider.get()
                        .create(Paths.Side.RIGHT, blueCloseLeftBranchBToLeftCoralStation)
                        .alongWith(getDriveAndIntakeStatusMessageCommand(
                                Landmarks.CoralStation.RIGHT,
                                Landmarks.CoralStationSection.MID));
        this.addCommands(driveToRightStationAndIntakeSecond);

        // Score 3
        var driveAndScoreCloseLeftBranchALevelFour =
                driveToFaceAndScoreFact
                        .create(Landmarks.ReefFace.CLOSE_RIGHT, Landmarks.Branch.B,
                                Landmarks.CoralLevel.FOUR)
                        .alongWith(getDriveAndScoreStatusMessageCommand(
                                Landmarks.ReefFace.CLOSE_RIGHT, Landmarks.Branch.B,
                                Landmarks.CoralLevel.FOUR));
        this.addCommands(driveAndScoreCloseLeftBranchALevelFour);

        // Coral Station
        var driveToLeftStationAndIntakeSecond =
                pathDriveToLocationAndIntakeUntilCollectedProvider.get()
                        .create(Paths.Side.RIGHT, blueCloseLeftBranchAToLeftCoralStation)
                        .alongWith(getDriveAndIntakeStatusMessageCommand(
                                Landmarks.CoralStation.RIGHT,
                                Landmarks.CoralStationSection.MID));
        this.addCommands(driveToLeftStationAndIntakeSecond);

        // Score 4
        var driveAndScoreCloseBranchALevelFour =
                driveToFaceAndScoreFact
                        .create(Landmarks.ReefFace.CLOSE, Landmarks.Branch.B,
                                Landmarks.CoralLevel.FOUR)
                        .alongWith(
                                getDriveAndScoreStatusMessageCommand(Landmarks.ReefFace.CLOSE,
                                        Landmarks.Branch.B, Landmarks.CoralLevel.FOUR));
        this.addCommands(driveAndScoreCloseBranchALevelFour);

        // Back to coral station
        var homed =
                pathDriveToLocationAndIntakeUntilCollectedProvider.get()
                        .create(Paths.Side.RIGHT, blueCloseBranchAToLeftCoralStation)
                        .alongWith(getDriveAndIntakeStatusMessageCommand(
                                Landmarks.CoralStation.RIGHT,
                                Landmarks.CoralStationSection.MID));
        this.addCommands(homed);
    }
}
