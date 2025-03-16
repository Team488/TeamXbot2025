package competition.auto_programs.vision;

import competition.auto_programs.BaseAutonomousSequentialCommandGroup;
import competition.commandgroups.DriveToFaceAndScoreCommandGroupFactory;
import competition.commandgroups.PrepCoralSystemCommandGroupFactory;
import competition.commandgroups.vision_path.PathDriveToLocationAndIntakeUntilCollected;
import competition.commandgroups.vision_path.PathToFaceAndScoreCommandGroupFactory;
import competition.simulation.BaseSimulator;
import competition.subsystems.pose.Landmarks;
import competition.subsystems.pose.PoseSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import org.kobe.xbot.Utilities.Entities.XTableValues;
import xbot.common.subsystems.autonomous.AutonomousCommandSelector;

import javax.inject.Inject;
import javax.inject.Provider;
import java.util.ArrayList;
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
    @Inject
    public LeftFourCoralAuto(AutonomousCommandSelector autoSelector,
                             PoseSubsystem pose,
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
        getDriveAndScoreStatusMessageCommand(Landmarks.ReefFace.FAR_LEFT, Landmarks.Branch.B, Landmarks.CoralLevel.FOUR);


        var driveAndScoreFarLeftBranchBLevelFour = driveToFaceAndScoreFact.create(
                        Landmarks.ReefFace.FAR_LEFT, Landmarks.Branch.B, Landmarks.CoralLevel.FOUR)
                .alongWith(getDriveAndScoreStatusMessageCommand(Landmarks.ReefFace.FAR_LEFT, Landmarks.Branch.B, Landmarks.CoralLevel.FOUR));
        this.addCommands(driveAndScoreFarLeftBranchBLevelFour);

        if(DriverStation.getAlliance().orElse(DriverStation.Alliance.Red).equals(DriverStation.Alliance.Blue)) {
            var driveToLeftStation = pathDriveToLocationAndIntakeUntilCollectedProvider.get().create(transformBezierCurve(farLeftBToCoralStation))
                    .alongWith(
                            getDriveAndIntakeStatusMessageCommand(Landmarks.CoralStation.LEFT, Landmarks.CoralStationSection.MID));
            this.addCommands(driveToLeftStation);
        } else {
            var driveToLeftStation = pathDriveToLocationAndIntakeUntilCollectedProvider.get().create(farLeftBToCoralStation)
                    .alongWith(
                            getDriveAndIntakeStatusMessageCommand(Landmarks.CoralStation.LEFT, Landmarks.CoralStationSection.MID));
            this.addCommands(driveToLeftStation);
        }



        var driveAndScoreCloseRightBranchBLevelFour = driveToFaceAndScoreFactProv.get().create(
                        Landmarks.ReefFace.CLOSE_LEFT, Landmarks.Branch.A, Landmarks.CoralLevel.FOUR, leftCoralStationToCloseLeftA)
                .alongWith(
                        getDriveAndScoreStatusMessageCommand(Landmarks.ReefFace.CLOSE_LEFT, Landmarks.Branch.A, Landmarks.CoralLevel.FOUR));
        this.addCommands(driveAndScoreCloseRightBranchBLevelFour);

//        // Drive to right coral station, close section and intake coral until collected
//        var driveToRightStationAndIntakeSecond = driveToStationAndIntakeFactProv.get().create()
//                .alongWith(
//                        getDriveAndIntakeStatusMessageCommand(Landmarks.CoralStation.LEFT, Landmarks.CoralStationSection.MID));
//        this.addCommands(driveToRightStationAndIntakeSecond);
//
//        // Drive to close right, branch A and score level four
//        var driveAndScoreCloseRightBranchALevelFour = driveToFaceAndScoreFactProv.get().create(
//                        Landmarks.ReefFace.CLOSE_LEFT, Landmarks.Branch.B, Landmarks.CoralLevel.FOUR)
//                .alongWith(
//                        getDriveAndScoreStatusMessageCommand(Landmarks.ReefFace.CLOSE_LEFT, Landmarks.Branch.B, Landmarks.CoralLevel.FOUR));
//        this.addCommands(driveAndScoreCloseRightBranchALevelFour);
//
//        // Drive to right coral station, far section and intake coral until collected again
//        var driveToRightStationAndIntakeThird = driveToStationAndIntakeFactProv.get().create()
//                .alongWith(
//                        getDriveAndIntakeStatusMessageCommand(Landmarks.CoralStation.LEFT, Landmarks.CoralStationSection.MID));
//        this.addCommands(driveToRightStationAndIntakeThird);
//
//        // Drive to close, branch A and score level four
//        var driveAndScoreCloseBranchALevelFour = driveToFaceAndScoreFactProv.get().create(
//                        Landmarks.ReefFace.CLOSE, Landmarks.Branch.A, Landmarks.CoralLevel.FOUR)
//                .alongWith(
//                        getDriveAndScoreStatusMessageCommand(Landmarks.ReefFace.CLOSE, Landmarks.Branch.A, Landmarks.CoralLevel.FOUR));
//        this.addCommands(driveAndScoreCloseBranchALevelFour);
//
//        var driveToRightStationAndIntakeFinal = driveToStationAndIntakeFactProv.get().create()
//                .alongWith(
//                        getDriveAndIntakeStatusMessageCommand(Landmarks.CoralStation.LEFT, Landmarks.CoralStationSection.MID));
//        this.addCommands(driveToRightStationAndIntakeFinal);
    }

    public static XTableValues.BezierCurves transformBezierCurve(XTableValues.BezierCurves curve) {
        XTableValues.BezierCurves.Builder updatedCurve = curve.toBuilder();

        for (int i = 0; i < updatedCurve.getCurvesCount(); i++) {
            XTableValues.BezierCurve.Builder updatedBezier = updatedCurve.getCurves(i).toBuilder();
            List<XTableValues.ControlPoint> updatedPoints = new ArrayList<>();

            for (XTableValues.ControlPoint point : updatedBezier.getControlPointsList()) {
                Pose2d transformedPose = PoseSubsystem.convertBluetoRed(new Pose2d(point.getX(), point.getY(),
                        new Rotation2d(0)));
                XTableValues.ControlPoint transformedPoint = XTableValues.ControlPoint.newBuilder()
                        .setX(transformedPose.getX())
                        .setY(transformedPose.getY())
                        .build();
                updatedPoints.add(transformedPoint);
            }

            updatedBezier.clearControlPoints();
            updatedBezier.addAllControlPoints(updatedPoints);
            updatedCurve.setCurves(i, updatedBezier);
        }

        return updatedCurve.build();
    }

}
