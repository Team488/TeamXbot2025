package competition.auto_programs;

import competition.commandgroups.DriveToFaceAndScoreCommandGroupFactory;
import competition.subsystems.pose.Landmarks;
import competition.subsystems.pose.PoseSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import xbot.common.subsystems.autonomous.AutonomousCommandSelector;

import javax.inject.Inject;

public class FromCageScoreOneCoralAutoFactory {

    final AutonomousCommandSelector autoSelector;
    PoseSubsystem pose;
    DriveToFaceAndScoreCommandGroupFactory driveToFaceAndScoreCommandGroupFact;

    @Inject
    public FromCageScoreOneCoralAutoFactory(AutonomousCommandSelector autoSelector,
                                            DriveToFaceAndScoreCommandGroupFactory driveToFaceAndScoreCommandGroupFact,
                                            PoseSubsystem pose){
        this.autoSelector = autoSelector;
        this.pose = pose;
        this.driveToFaceAndScoreCommandGroupFact = driveToFaceAndScoreCommandGroupFact;
    }

    public BaseAutonomousSequentialCommandGroup create(Pose2d startingLocation,
                                         Landmarks.ReefFace targetReefFace, Landmarks.Branch targetBranch,
                                         Landmarks.CoralLevel targetLevel) {
        var auto = new BaseAutonomousSequentialCommandGroup(autoSelector);

        var startInFrontOfCage = pose.createSetPositionCommand(PoseSubsystem.convertBlueToRedIfNeeded(startingLocation));
        auto.addCommands(startInFrontOfCage);

        auto.queueDriveAndScoreMessageToAutoSelector(targetReefFace, targetBranch, targetLevel);
        var driveAndScore = driveToFaceAndScoreCommandGroupFact.create(targetReefFace, targetBranch, targetLevel);
        auto.addCommands(driveAndScore);

        return auto;
    }


}
