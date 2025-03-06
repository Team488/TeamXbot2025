package competition.auto_programs;

import competition.commandgroups.DriveToFaceAndScoreCommandGroupFactory;
import competition.commandgroups.PrepCoralSystemCommandGroupFactory;
import competition.simulation.BaseSimulator;
import competition.simulation.MapleSimulator;
import competition.subsystems.pose.Landmarks;
import competition.subsystems.pose.PoseSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import xbot.common.command.BaseSequentialCommandGroup;
import xbot.common.subsystems.autonomous.AutonomousCommandSelector;

import javax.inject.Inject;
import javax.inject.Provider;

public class FromCageScoreOneCoralAutoFactory {

    final AutonomousCommandSelector autoSelector;
    BaseSimulator simulator;
    PoseSubsystem pose;
    DriveToFaceAndScoreCommandGroupFactory driveToFaceAndScoreCommandGroupFact;
    PrepCoralSystemCommandGroupFactory prepCoralSystemCommandGroupFact;

    @Inject
    public FromCageScoreOneCoralAutoFactory(AutonomousCommandSelector autoSelector,
                                            BaseSimulator simulator,
                                            PoseSubsystem pose,
                                            DriveToFaceAndScoreCommandGroupFactory driveToFaceAndScoreCommandGroupFact,
                                            PrepCoralSystemCommandGroupFactory prepCoralSystemCommandGroupFact){
        this.autoSelector = autoSelector;
        this.simulator = simulator;
        this.pose = pose;
        this.driveToFaceAndScoreCommandGroupFact = driveToFaceAndScoreCommandGroupFact;
        this.prepCoralSystemCommandGroupFact = prepCoralSystemCommandGroupFact;
    }

    public BaseAutonomousSequentialCommandGroup create(Pose2d startingLocation,
                                         Landmarks.ReefFace targetReefFace, Landmarks.Branch targetBranch,
                                         Landmarks.CoralLevel targetLevel) {
        var auto = new BaseAutonomousSequentialCommandGroup(autoSelector);
        auto.setName("FromCageScoreOneCoralAuto");

        var startInFrontOfCage = pose.createSetPositionCommand(PoseSubsystem.convertBlueToRedIfNeeded(startingLocation));
        auto.addCommands(startInFrontOfCage);

        var resetSim = new InstantCommand(() -> simulator.resetPosition(PoseSubsystem.convertBlueToRedIfNeeded(startingLocation)));
        auto.addCommands(resetSim);

        auto.queueDriveAndScoreMessageToAutoSelector(targetReefFace, targetBranch, targetLevel);
        var driveAndScore = driveToFaceAndScoreCommandGroupFact.create(targetReefFace, targetBranch, targetLevel);
        auto.addCommands(driveAndScore);

        var homeCoralSystem = prepCoralSystemCommandGroupFact.create(() -> Landmarks.CoralLevel.COLLECTING);
        auto.addCommands(homeCoralSystem);

        return auto;
    }


}
