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


public class TestPathPlannerCommand extends SequentialCommandGroup {

    Logger log;
    @Inject
    public TestPathPlannerCommand(PoseSubsystem pose, Provider<FollowPathCommand> followPathCommandProvider) {

        var path1 = followPathCommandProvider.get();
        try {
            path1.setPath(PathPlannerPath.fromPathFile("Test Path"));
        } catch (ParseException | IOException e) {
            log.info("Cant find path");
            cancel();
        }

//        //TODO: convert Blue to Red if needed
//        var startingPose = pose.createSetPositionCommand(scoreCoral1.getPathStartingPose());
//
//        this.addCommands(startingPose);
        this.addCommands(path1);


    }
}