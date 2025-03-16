package competition.auto_programs.vision;

import competition.auto_programs.BaseAutonomousSequentialCommandGroup;
import competition.commandgroups.DriveToFaceAndScoreCommandGroupFactory;
import competition.commandgroups.PrepCoralSystemCommandGroupFactory;
import competition.commandgroups.vision_path.PathDriveToCoralStationAndIntakeUntilCollected;
import competition.commandgroups.vision_path.PathDriveToLocationAndIntakeUntilCollected;
import competition.commandgroups.vision_path.PathToFaceAndScoreCommandGroupFactory;
import competition.electrical_contract.ElectricalContract;
import competition.simulation.BaseSimulator;
import competition.subsystems.pose.Landmarks;
import competition.subsystems.pose.PoseSubsystem;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import org.kobe.xbot.Utilities.Entities.XTableValues;
import xbot.common.subsystems.autonomous.AutonomousCommandSelector;

import javax.inject.Inject;
import javax.inject.Provider;
import java.util.List;


public class LeftFourCoralAuto extends BaseAutonomousSequentialCommandGroup {
    XTableValues.BezierCurves farLeftBToCoralStation = XTableValues.BezierCurves.newBuilder()
            .setOptions(XTableValues.TraversalOptions.newBuilder()
                    .setFinalRotationDegrees(126)
                    .build())
            .addCurves(0, XTableValues.BezierCurve.newBuilder()
                    .addAllControlPoints(List.of(
                            XTableValues.ControlPoint.newBuilder()
                                    .setX(12.53934783)
                                    .setY(2.82768987)
                                    .build(),
                            XTableValues.ControlPoint.newBuilder()
                                    .setX(14.34521739)
                                    .setY(1.01898734)
                                    .build(),
                            XTableValues.ControlPoint.newBuilder()
                                    .setX(16.40543478)
                                    .setY(1.01898734)
                                    .build()



                    ))
                    .build())
            .build();
    XTableValues.BezierCurves leftCoralStationToCloseLeftA = XTableValues.BezierCurves.newBuilder()
            .setOptions(XTableValues.TraversalOptions.newBuilder()
                    .setFinalRotationDegrees(126)
                    .build())
            .addCurves(0, XTableValues.BezierCurve.newBuilder()
                    .addAllControlPoints(List.of(
                            XTableValues.ControlPoint.newBuilder()
                                    .setX(16.405434782608697)
                                    .setY(1.018987341772152)
                                    .build(),
                            XTableValues.ControlPoint.newBuilder()
                                    .setX(14.599565217391307)
                                    .setY(2.8276898734177216)
                                    .build(),
                            XTableValues.ControlPoint.newBuilder()
                                    .setX(13.556739130434785)
                                    .setY(2.8276898734177216)
                                    .build()



                    ))
                    .build())
            .build();
    XTableValues.BezierCurves blueCageOneStartingLineToFarLeftBranchA = XTableValues.BezierCurves.newBuilder()
            .setOptions(XTableValues.TraversalOptions.newBuilder()
                    .setFinalRotationDegrees(-120.00)
                    .build())
            .addCurves(0, XTableValues.BezierCurve.newBuilder()
                    .addAllControlPoints(List.of(
                            XTableValues.ControlPoint.newBuilder()
                                    .setX(7.1217391304347837)
                                    .setY(7.2348101265822793)
                                    .build(),
                            XTableValues.ControlPoint.newBuilder()
                                    .setX(5.0869565217391308)
                                    .setY(5.1968354430379753)
                                    .build(),
                            XTableValues.ControlPoint.newBuilder()
                                    .setX(4.9852173913043485)
                                    .setY(5.1968354430379753)
                                    .build()



                    ))
                    .build())
            .build();
    XTableValues.BezierCurves farLeftAToCoralStationBlue = XTableValues.BezierCurves.newBuilder()
            .setOptions(XTableValues.TraversalOptions.newBuilder()
                    .setFinalRotationDegrees(-54)
                    .build())
            .addCurves(0, XTableValues.BezierCurve.newBuilder()
                    .addAllControlPoints(List.of(
                            XTableValues.ControlPoint.newBuilder()
                                    .setX(4.9852173913043485)
                                    .setY(5.1968354430379753)
                                    .build(),
                            XTableValues.ControlPoint.newBuilder()
                                    .setX(3.179347826086957)
                                    .setY(7.0055379746835449)
                                    .build(),
                            XTableValues.ControlPoint.newBuilder()
                                    .setX(1.1191304347826088)
                                    .setY(7.0055379746835449)
                                    .build()



                    ))
                    .build())
            .build();
    XTableValues.BezierCurves blueLeftCoralStationToCloseLeftBranchA = XTableValues.BezierCurves.newBuilder()
            .setOptions(XTableValues.TraversalOptions.newBuilder()
                    .setFinalRotationDegrees(-60.00)
                    .build())
            .addCurves(0, XTableValues.BezierCurve.newBuilder()
                    .addAllControlPoints(List.of(
                            XTableValues.ControlPoint.newBuilder()
                                    .setX(1.1191304347826088)
                                    .setY(7.0055379746835449)
                                    .build(),
                            XTableValues.ControlPoint.newBuilder()
                                    .setX(2.9250000000000003)
                                    .setY(5.1968354430379753)
                                    .build(),
                            XTableValues.ControlPoint.newBuilder()
                                    .setX(3.9678260869565221)
                                    .setY(5.1968354430379753)
                                    .build()
                    ))
                    .build())
            .build();
    XTableValues.BezierCurves blueCloseLeftBranchAToLeftCoralStation = XTableValues.BezierCurves.newBuilder()
            .setOptions(XTableValues.TraversalOptions.newBuilder()
                    .setFinalRotationDegrees(-54)
                    .build())
            .addCurves(0, XTableValues.BezierCurve.newBuilder()
                    .addAllControlPoints(List.of(
                            XTableValues.ControlPoint.newBuilder()
                                    .setX(3.9678260869565221)
                                    .setY(5.1968354430379753)
                                    .build(),
                            XTableValues.ControlPoint.newBuilder()
                                    .setX(2.1619565217391306)
                                    .setY(7.0055379746835449)
                                    .build(),
                            XTableValues.ControlPoint.newBuilder()
                                    .setX(1.1191304347826088)
                                    .setY(7.0055379746835449)
                                    .build()
                    ))
                    .build())
            .build();
    XTableValues.BezierCurves blueLeftCoralStationToCloseBranchA = XTableValues.BezierCurves.newBuilder()
            .setOptions(XTableValues.TraversalOptions.newBuilder()
                    .setFinalRotationDegrees(0)
                    .build())
            .addCurves(0, XTableValues.BezierCurve.newBuilder()
                    .addAllControlPoints(List.of(
                            XTableValues.ControlPoint.newBuilder()
                                    .setX(1.1191304347826088)
                                    .setY(7.0055379746835449)
                                    .build(),

                            XTableValues.ControlPoint.newBuilder()
                                    .setX(3.179347826086957)
                                    .setY(4.9420886075949371)
                                    .build(),
                            XTableValues.ControlPoint.newBuilder()
                                    .setX(3.179347826086957)
                                    .setY(4.1778481012658233)
                                    .build()
                    ))
                    .build())
            .build();
    XTableValues.BezierCurves blueCloseBranchAToLeftCoralStation = XTableValues.BezierCurves.newBuilder()
            .setOptions(XTableValues.TraversalOptions.newBuilder()
                    .setFinalRotationDegrees(-54)
                    .build())
            .addCurves(0, XTableValues.BezierCurve.newBuilder()
                    .addAllControlPoints(List.of(
                            XTableValues.ControlPoint.newBuilder()
                                    .setX(3.179347826086957)
                                    .setY(4.1778481012658233)
                                    .build(),

                            XTableValues.ControlPoint.newBuilder()
                                    .setX(3.179347826086957)
                                    .setY(4.9420886075949371)
                                    .build(),
                            XTableValues.ControlPoint.newBuilder()
                                    .setX(1.1191304347826088)
                                    .setY(7.0055379746835449)
                                    .build()
                    ))
                    .build())
            .build();
    @Inject
    public LeftFourCoralAuto(
            AprilTagFieldLayout aprilTagFieldLayout,
            ElectricalContract electricalContract,
            AutonomousCommandSelector autoSelector,
                             PoseSubsystem pose,
                             Provider<PathDriveToCoralStationAndIntakeUntilCollected> driveToStationAndIntakeFactProv,
                             DriveToFaceAndScoreCommandGroupFactory driveToFaceAndScoreFact,
                             Provider<PathToFaceAndScoreCommandGroupFactory> driveToFaceAndScoreFactProv,
                             Provider<PathDriveToLocationAndIntakeUntilCollected> pathDriveToLocationAndIntakeUntilCollectedProvider,
                             PrepCoralSystemCommandGroupFactory prepCoralSystemCommandGroupFact,
                             BaseSimulator simulator) {

        super(autoSelector);
        var initializeStateCommand = pose.createSetPositionCommand(
                        () -> PoseSubsystem.convertBlueToRedIfNeeded(Landmarks.BlueCageOneStartingLine)
                )
                .alongWith(new InstantCommand(() -> simulator.resetPosition(PoseSubsystem.convertBlueToRedIfNeeded(Landmarks.BlueCageOneStartingLine))));
        this.addCommands(initializeStateCommand);

        // Score 1

//        getDriveAndScoreStatusMessageCommand(Landmarks.ReefFace.FAR_LEFT, Landmarks.Branch.B, Landmarks.CoralLevel.FOUR);
//        var driveAndScoreFarRightBranchBLevelFour = driveToFaceAndScoreFactProv.get().create(
//                        Landmarks.ReefFace.FAR_LEFT, Landmarks.Branch.B, Landmarks.CoralLevel.FOUR, blueCageOneStartingLineToFarLeftBranchA)
//                .alongWith(getDriveAndScoreStatusMessageCommand(Landmarks.ReefFace.FAR_LEFT, Landmarks.Branch.B, Landmarks.CoralLevel.FOUR));
//        this.addCommands(driveAndScoreFarRightBranchBLevelFour);
        getDriveAndScoreStatusMessageCommand(Landmarks.ReefFace.FAR_LEFT, Landmarks.Branch.B, Landmarks.CoralLevel.FOUR);
        var driveAndScoreFarLeftBranchBLevelFour = driveToFaceAndScoreFact.create(
                        Landmarks.ReefFace.FAR_LEFT, Landmarks.Branch.B, Landmarks.CoralLevel.FOUR)
                .alongWith(getDriveAndScoreStatusMessageCommand(Landmarks.ReefFace.FAR_LEFT, Landmarks.Branch.B, Landmarks.CoralLevel.FOUR));
        this.addCommands(driveAndScoreFarLeftBranchBLevelFour);

        // Coral Station
        var driveToLeftStation = pathDriveToLocationAndIntakeUntilCollectedProvider.get().create(farLeftAToCoralStationBlue)
                .alongWith(
                        getDriveAndIntakeStatusMessageCommand(Landmarks.CoralStation.LEFT, Landmarks.CoralStationSection.MID));
        this.addCommands(driveToLeftStation);

        // Score 2
//        var driveAndScoreCloseRightBranchBLevelFour = driveToFaceAndScoreFactProv.get().create(
//                        Landmarks.ReefFace.CLOSE_LEFT, Landmarks.Branch.A, Landmarks.CoralLevel.FOUR, blueLeftCoralStationToCloseLeftBranchA)
//                .alongWith(
//                        getDriveAndScoreStatusMessageCommand(Landmarks.ReefFace.CLOSE_LEFT, Landmarks.Branch.A, Landmarks.CoralLevel.FOUR));
//        this.addCommands(driveAndScoreCloseRightBranchBLevelFour);
        var driveAndScoreCloseLeftBranchBLevelFour = driveToFaceAndScoreFact.create(
                        Landmarks.ReefFace.CLOSE_LEFT, Landmarks.Branch.B, Landmarks.CoralLevel.FOUR)
                .alongWith(getDriveAndScoreStatusMessageCommand(Landmarks.ReefFace.CLOSE_LEFT, Landmarks.Branch.B, Landmarks.CoralLevel.FOUR));
        this.addCommands(driveAndScoreCloseLeftBranchBLevelFour);

        // Coral Station
        var driveToRightStationAndIntakeSecond = pathDriveToLocationAndIntakeUntilCollectedProvider.get().create(blueCloseLeftBranchAToLeftCoralStation)
                .alongWith(
                        getDriveAndIntakeStatusMessageCommand(Landmarks.CoralStation.LEFT, Landmarks.CoralStationSection.MID));
        this.addCommands(driveToRightStationAndIntakeSecond);


        // Score 3
//        var driveAndScoreCloseaBranchALevelFour = driveToFaceAndScoreFactProv.get().create(
//                        Landmarks.ReefFace.CLOSE_LEFT, Landmarks.Branch.B, Landmarks.CoralLevel.FOUR, blueLeftCoralStationToCloseLeftBranchA)
//                .alongWith(
//                        getDriveAndScoreStatusMessageCommand(Landmarks.ReefFace.CLOSE_LEFT, Landmarks.Branch.B, Landmarks.CoralLevel.FOUR));
//        this.addCommands(driveAndScoreCloseaBranchALevelFour);
        var driveAndScoreCloseLeftBranchALevelFour = driveToFaceAndScoreFact.create(
                        Landmarks.ReefFace.CLOSE_LEFT, Landmarks.Branch.A, Landmarks.CoralLevel.FOUR)
                .alongWith(
                        getDriveAndScoreStatusMessageCommand(Landmarks.ReefFace.CLOSE_LEFT, Landmarks.Branch.A, Landmarks.CoralLevel.FOUR));
        this.addCommands(driveAndScoreCloseLeftBranchALevelFour);

        // Coral Station
        var driveToLeftStationAndIntakeSecond = pathDriveToLocationAndIntakeUntilCollectedProvider.get().create(blueCloseLeftBranchAToLeftCoralStation)
                .alongWith(
                        getDriveAndIntakeStatusMessageCommand(Landmarks.CoralStation.LEFT, Landmarks.CoralStationSection.MID));
        this.addCommands(driveToLeftStationAndIntakeSecond);

        // Score 4
//        var driveAndScoreCloseBranchALevelFour = driveToFaceAndScoreFactProv.get().create(
//                        Landmarks.ReefFace.CLOSE, Landmarks.Branch.A, Landmarks.CoralLevel.FOUR, blueLeftCoralStationToCloseBranchA)
//                .alongWith(
//                        getDriveAndScoreStatusMessageCommand(Landmarks.ReefFace.CLOSE, Landmarks.Branch.A, Landmarks.CoralLevel.FOUR));
//        this.addCommands(driveAndScoreCloseBranchALevelFour);
        var driveAndScoreCloseBranchALevelFour = driveToFaceAndScoreFact.create(
                        Landmarks.ReefFace.CLOSE, Landmarks.Branch.A, Landmarks.CoralLevel.FOUR)
                .alongWith(
                        getDriveAndScoreStatusMessageCommand(Landmarks.ReefFace.CLOSE, Landmarks.Branch.A, Landmarks.CoralLevel.FOUR));
        this.addCommands(driveAndScoreCloseBranchALevelFour);

        // Home
        var homed = pathDriveToLocationAndIntakeUntilCollectedProvider.get().create(blueCloseBranchAToLeftCoralStation)
                .alongWith(
                        getDriveAndIntakeStatusMessageCommand(Landmarks.CoralStation.LEFT, Landmarks.CoralStationSection.MID));
        this.addCommands(homed);
    }
}
