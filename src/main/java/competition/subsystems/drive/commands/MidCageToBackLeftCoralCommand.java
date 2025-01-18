package competition.subsystems.drive.commands;


import com.pathplanner.lib.path.PathPlannerPath;
import competition.subsystems.pose.PoseSubsystem;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import org.json.simple.parser.ParseException;
import xbot.common.advantage.AKitLogger;

import javax.inject.Inject;
import javax.inject.Provider;
import java.io.IOException;
import java.util.logging.Logger;


public class MidCageToBackLeftCoralCommand extends SequentialCommandGroup {

    Logger log;
    @Inject
    public MidCageToBackLeftCoralCommand(PoseSubsystem pose, Provider<FollowPathCommand> followPathCommandProvider) {

        var scoreCoral1 = followPathCommandProvider.get();
        try {
            scoreCoral1.setPath(PathPlannerPath.fromPathFile("MIDCAGE_TO_BACKLEFT"));
        } catch (ParseException | IOException e) {
            log.info("Cant find path");
            cancel();
        }

        //TODO: convert Blue to Red if needed
        var startingPose = pose.createSetPositionCommand(scoreCoral1.getStartingPose());

        this.addCommands(startingPose);
        this.addCommands(scoreCoral1);

    }
}