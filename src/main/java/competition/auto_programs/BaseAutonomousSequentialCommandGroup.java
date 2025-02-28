package competition.auto_programs;

import competition.subsystems.pose.Landmarks;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import xbot.common.subsystems.autonomous.AutonomousCommandSelector;

public class BaseAutonomousSequentialCommandGroup extends SequentialCommandGroup {

    final AutonomousCommandSelector autoSelector;

    public BaseAutonomousSequentialCommandGroup(AutonomousCommandSelector autoSelector) {
        this.autoSelector = autoSelector;
    }

    public void queueMessageToAutoSelector(String message) {
        this.addCommands(autoSelector.createAutonomousStateMessageCommand(message));
    }

    public void queueDriveAndScoreMessageToAutoSelector(Landmarks.ReefFace targetReefFace,
                                                        Landmarks.Branch targetBranch,
                                                        Landmarks.CoralLevel targetLevel) {
        String message = "Drive to " + targetReefFace + ", Branch " + targetBranch + ", and score" + targetLevel;
        queueMessageToAutoSelector(message);
    }

    public void queueDriveAndIntakeMessageToAutoSelector(Landmarks.CoralStation targetStation,
                                                         Landmarks.CoralStationSection targetSection) {
        String message = "Drive to " + targetStation + " coral station, " + targetSection + " section and intake coral until collected";
    }

}
