package competition.auto_programs;

import competition.commandgroups.DriveToFaceAndScoreCommandGroupFactory;
import competition.simulation.MapleSimulator;
import competition.subsystems.pose.Landmarks;
import competition.subsystems.pose.PoseSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import xbot.common.subsystems.autonomous.AutonomousCommandSelector;

import javax.inject.Inject;

public class FromCageScoreOneCoralAutoFactory {

    final AutonomousCommandSelector autoSelector;
    MapleSimulator mapleSimulator;
    PoseSubsystem pose;
    DriveToFaceAndScoreCommandGroupFactory driveToFaceAndScoreCommandGroupFact;

    @Inject
    public FromCageScoreOneCoralAutoFactory(AutonomousCommandSelector autoSelector,
                                            MapleSimulator mapleSimulator,
                                            PoseSubsystem pose,
                                            DriveToFaceAndScoreCommandGroupFactory driveToFaceAndScoreCommandGroupFact){
        this.autoSelector = autoSelector;
        this.mapleSimulator = mapleSimulator;
        this.pose = pose;
        this.driveToFaceAndScoreCommandGroupFact = driveToFaceAndScoreCommandGroupFact;
    }

    public BaseAutonomousSequentialCommandGroup create(Pose2d startingLocation,
                                         Landmarks.ReefFace targetReefFace, Landmarks.Branch targetBranch,
                                         Landmarks.CoralLevel targetLevel) {
        var auto = new BaseAutonomousSequentialCommandGroup(autoSelector);

        var startInFrontOfCage = pose.createSetPositionCommand(PoseSubsystem.convertBlueToRedIfNeeded(startingLocation));
        auto.addCommands(startInFrontOfCage);

        var resetMapleSim = new InstantCommand(() -> mapleSimulator.resetPosition(PoseSubsystem.convertBlueToRedIfNeeded(startingLocation)));
        auto.addCommands(resetMapleSim);

        auto.queueDriveAndScoreMessageToAutoSelector(targetReefFace, targetBranch, targetLevel);
        var driveAndScore = driveToFaceAndScoreCommandGroupFact.create(targetReefFace, targetBranch, targetLevel);
        auto.addCommands(driveAndScore);

        return auto;
    }


}
