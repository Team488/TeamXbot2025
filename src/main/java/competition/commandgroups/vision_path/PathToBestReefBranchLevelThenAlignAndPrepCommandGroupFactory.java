package competition.commandgroups.vision_path;

import static competition.subsystems.vision.CoprocessorCommunicationSubsystem.fromAlliance;
import static edu.wpi.first.units.Units.Meters;

import competition.commandgroups.PrepCoralSystemCommandGroupFactory;
import competition.subsystems.coral_scorer.commands.ScoreWhenReadyCommand;
import competition.subsystems.drive.commands.AlignToTagGlobalMovementWithCalculator;
import competition.subsystems.drive.commands.MeasureDistanceBeforeScoringCommand;
import competition.subsystems.drive.commands.vision_path.PathDriveToLocationCommandUntilAprilTagDetectionDynamic;
import competition.subsystems.pose.Cameras;
import competition.subsystems.pose.Landmarks;
import competition.subsystems.vision.AprilTagVisionSubsystemExtended;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import javax.inject.Inject;
import javax.inject.Provider;
import org.kobe.xbot.Utilities.Entities.XTableValues;

public class PathToBestReefBranchLevelThenAlignAndPrepCommandGroupFactory {
    Provider<PathDriveToLocationCommandUntilAprilTagDetectionDynamic>
            driveToReefFaceCommandUntilAprilTagDetection;
    AlignToTagGlobalMovementWithCalculator alignToReefWithAprilTagCommand;
    AprilTagVisionSubsystemExtended aprilTagVisionSubsystem;
    PrepCoralSystemCommandGroupFactory prepCoralSystemFactory;
    Provider<ScoreWhenReadyCommand> scoreWhenReadyProvider;
    MeasureDistanceBeforeScoringCommand measureDistanceBeforeScoringCommand;

    @Inject
    public PathToBestReefBranchLevelThenAlignAndPrepCommandGroupFactory(
            Provider<PathDriveToLocationCommandUntilAprilTagDetectionDynamic>
                    driveToReefFaceCommandUntilAprilTagDetection,
            AlignToTagGlobalMovementWithCalculator alignToReefWithAprilTagCommand,
            AprilTagVisionSubsystemExtended aprilTagVisionSubsystem,
            PrepCoralSystemCommandGroupFactory prepCoralSystemFactory,
            Provider<ScoreWhenReadyCommand> scoreWhenReadyProvider,
            MeasureDistanceBeforeScoringCommand measureDistanceBeforeScoringCommand) {
        this.driveToReefFaceCommandUntilAprilTagDetection =
                driveToReefFaceCommandUntilAprilTagDetection;
        this.alignToReefWithAprilTagCommand = alignToReefWithAprilTagCommand;
        this.aprilTagVisionSubsystem = aprilTagVisionSubsystem;
        this.prepCoralSystemFactory = prepCoralSystemFactory;
        this.scoreWhenReadyProvider = scoreWhenReadyProvider;
        this.measureDistanceBeforeScoringCommand =
                measureDistanceBeforeScoringCommand;
    }

    public SequentialCommandGroup create() {
        PathDriveToLocationCommandUntilAprilTagDetectionDynamic
                pathDriveToLocationUntilAprilTagDetectionDynamic =
                driveToReefFaceCommandUntilAprilTagDetection.get();
        return new SequentialCommandGroup(
                // Step 1: Configure and Run Path Drive
                new InstantCommand(() -> {
                    // This will auto get overwritten by XTABLES Vision Coprocessor
                    // (ORIN).
                    pathDriveToLocationUntilAprilTagDetectionDynamic.setTarget(
                            new Pose2d());
                    pathDriveToLocationUntilAprilTagDetectionDynamic
                            .setAdditionalArguments(
                                    XTableValues.AdditionalArguments.newBuilder()
                                            .setAlliance(
                                                    fromAlliance(DriverStation.getAlliance().orElse(
                                                            DriverStation.Alliance.Blue)))
                                            .setGoalToBestReefBranch(true)
                                            .build());
                }),
                pathDriveToLocationUntilAprilTagDetectionDynamic,

                // Step 2: Extract data after driving is done
                new InstantCommand(() -> {
                    XTableValues.BezierCurves curves = pathDriveToLocationUntilAprilTagDetectionDynamic.curves.get();
                    if (curves != null && curves.hasAlignToReefAprilTagOptions()) {
                        int cameraIndex =
                                curves
                                        .getAlignToReefAprilTagOptions()
                                        .getCamera()
                                        .equals(XTableValues.AprilTagCamera.FRONT_LEFT)
                                        ? Cameras.FRONT_LEFT_CAMERA.getIndex()
                                        : Cameras.FRONT_RIGHT_CAMERA.getIndex();

                        alignToReefWithAprilTagCommand.setConfigurations(cameraIndex,
                                curves
                                        .getAlignToReefAprilTagOptions()
                                        .getAprilTagID(),
                                false, -2);
                        measureDistanceBeforeScoringCommand.setDistanceThreshold(
                                Meters.of(1));
                    } else {
                        alignToReefWithAprilTagCommand.cancel();
                    }
                }),
                new ParallelCommandGroup(alignToReefWithAprilTagCommand,
                        new SequentialCommandGroup(measureDistanceBeforeScoringCommand,
                                prepCoralSystemFactory.create(
                                        ()
                                                -> toCoralLevel(
                                                pathDriveToLocationUntilAprilTagDetectionDynamic
                                                        .curves.get().getAlignToReefAprilTagOptions()
                                                        .getBranchLevel())),
                                scoreWhenReadyProvider.get())));
    }

    private Landmarks.CoralLevel toCoralLevel(
            XTableValues.BranchLevel branchLevel) {
        return switch (branchLevel) {
            case TROUGH, UNRECOGNIZED -> Landmarks.CoralLevel.ONE;

            case LEVEL_2 -> Landmarks.CoralLevel.TWO;

            case LEVEL_3 -> Landmarks.CoralLevel.THREE;
            case LEVEL_4 -> Landmarks.CoralLevel.FOUR;
        };
    }

}
