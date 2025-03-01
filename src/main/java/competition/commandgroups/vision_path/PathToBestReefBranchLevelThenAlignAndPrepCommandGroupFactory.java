package competition.commandgroups.vision_path;

import competition.commandgroups.PrepCoralSystemCommandGroupFactory;
import competition.subsystems.coral_scorer.commands.ScoreWhenReadyCommand;
import competition.subsystems.drive.commands.AlignToTagGlobalMovementWithCalculator;
import competition.subsystems.drive.commands.MeasureDistanceBeforeScoringCommand;
import competition.subsystems.drive.commands.vision_path.PathDriveToLocationUntilAprilTagDetectionDynamic;
import competition.subsystems.pose.Cameras;
import competition.subsystems.pose.Landmarks;
import competition.subsystems.vision.AprilTagVisionSubsystemExtended;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.kobe.xbot.Utilities.Entities.XTableValues;

import javax.inject.Inject;
import javax.inject.Provider;

import static edu.wpi.first.units.Units.Meters;

public class PathToBestReefBranchLevelThenAlignAndPrepCommandGroupFactory {

    PathDriveToLocationUntilAprilTagDetectionDynamic driveToReefFaceCommandUntilAprilTagDetection;
    AlignToTagGlobalMovementWithCalculator alignToReefWithAprilTagCommand;
    AprilTagVisionSubsystemExtended aprilTagVisionSubsystem;
    PrepCoralSystemCommandGroupFactory prepCoralSystemFactory;
    Provider<ScoreWhenReadyCommand> scoreWhenReadyProvider;
    MeasureDistanceBeforeScoringCommand measureDistanceBeforeScoringCommand;

    @Inject
    public PathToBestReefBranchLevelThenAlignAndPrepCommandGroupFactory(
            PathDriveToLocationUntilAprilTagDetectionDynamic driveToReefFaceCommandUntilAprilTagDetection,
            AlignToTagGlobalMovementWithCalculator alignToReefWithAprilTagCommand,
            AprilTagVisionSubsystemExtended aprilTagVisionSubsystem,
            PrepCoralSystemCommandGroupFactory prepCoralSystemFactory,
            Provider<ScoreWhenReadyCommand> scoreWhenReadyProvider,
            MeasureDistanceBeforeScoringCommand measureDistanceBeforeScoringCommand) {
        this.driveToReefFaceCommandUntilAprilTagDetection = driveToReefFaceCommandUntilAprilTagDetection;
        this.alignToReefWithAprilTagCommand = alignToReefWithAprilTagCommand;
        this.aprilTagVisionSubsystem = aprilTagVisionSubsystem;
        this.prepCoralSystemFactory = prepCoralSystemFactory;
        this.scoreWhenReadyProvider = scoreWhenReadyProvider;
        this.measureDistanceBeforeScoringCommand = measureDistanceBeforeScoringCommand;
    }

    public SequentialCommandGroup create() {
        return new SequentialCommandGroup(
                // Step 1: Configure and Run Path Drive
                new InstantCommand(() -> {
                    driveToReefFaceCommandUntilAprilTagDetection.setTarget(new Pose2d());
                    driveToReefFaceCommandUntilAprilTagDetection.setAdditionalArguments(XTableValues.AdditionalArguments.newBuilder()
                                    .setAlliance(fromAlliance(DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)))
                                    .setGoalToBestReefBranch(true)
                            .build());
                }),
                driveToReefFaceCommandUntilAprilTagDetection,

                // Step 2: Extract data after driving is done
                new InstantCommand(() -> {
                    if (driveToReefFaceCommandUntilAprilTagDetection.curves.hasAlignToReefAprilTagOptions()) {
                        int cameraIndex = driveToReefFaceCommandUntilAprilTagDetection.curves.getAlignToReefAprilTagOptions().getCamera()
                                .equals(XTableValues.AprilTagCamera.FRONT_LEFT) ? Cameras.FRONT_LEFT_CAMERA.getIndex() :
                                Cameras.FRONT_RIGHT_CAMERA.getIndex();

                        alignToReefWithAprilTagCommand.setConfigurations(cameraIndex,
                                driveToReefFaceCommandUntilAprilTagDetection.curves.getAlignToReefAprilTagOptions()
                                        .getAprilTagID(),false, -2);
                        measureDistanceBeforeScoringCommand.setDistanceThreshold(Meters.of(1));
                    } else {
                        alignToReefWithAprilTagCommand.end(true); // Not sure if it cancels it or not.
                    }
                }),
                new ParallelCommandGroup(alignToReefWithAprilTagCommand,
                        new SequentialCommandGroup(measureDistanceBeforeScoringCommand,
                                prepCoralSystemFactory.create(() ->
                                        toCoralLevel(driveToReefFaceCommandUntilAprilTagDetection.curves
                        .getAlignToReefAprilTagOptions().getBranchLevel())),
                        scoreWhenReadyProvider.get()))
        );
    }


    private Landmarks.CoralLevel toCoralLevel(XTableValues.BranchLevel branchLevel) {
        return switch (branchLevel) {
            case TROUGH, UNRECOGNIZED -> Landmarks.CoralLevel.ONE;

            case LEVEL_2 -> Landmarks.CoralLevel.TWO;

            case LEVEL_3 -> Landmarks.CoralLevel.THREE;
            case LEVEL_4 -> Landmarks.CoralLevel.FOUR;
        };
    }

    private XTableValues.Alliance fromAlliance(DriverStation.Alliance alliance) {
        return alliance.equals(DriverStation.Alliance.Blue) ? XTableValues.Alliance.BLUE : XTableValues.Alliance.RED;
    }
}
