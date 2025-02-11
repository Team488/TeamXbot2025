package competition.subsystems.oracle;

import competition.subsystems.pose.Landmarks;

import javax.inject.Inject;
import javax.inject.Singleton;
import javax.swing.text.html.Option;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import java.util.Queue;

@Singleton
public class ScoringQueue {

    private final Queue<ScoringTask> scoringTasks;
    private boolean defaultState = true;

    @Inject
    public ScoringQueue() {
        this.scoringTasks = new LinkedList<>();

        // TODO: set this up based on some commands invoked by the operator (before auto, or during the match)
        // for now, cheating in some tasks.

        addCoralTask(Landmarks.ReefFace.CLOSE_LEFT, Landmarks.Branch.A, Landmarks.CoralLevel.FOUR);
        addCoralTask(Landmarks.ReefFace.CLOSE_LEFT, Landmarks.Branch.B, Landmarks.CoralLevel.FOUR);

        addCoralTask(Landmarks.ReefFace.CLOSE_LEFT, Landmarks.Branch.A, Landmarks.CoralLevel.THREE);
        addCoralTask(Landmarks.ReefFace.CLOSE_LEFT, Landmarks.Branch.B, Landmarks.CoralLevel.THREE);
    }

    public void addCoralTask(Landmarks.ReefFace face, Landmarks.Branch branch, Landmarks.CoralLevel level) {
        scoringTasks.add(new ScoringTask(
                GameAction.ScoreCoral,
                Optional.of(face),
                Optional.of(branch),
                Optional.of(level)));
    }

    public void addAlgaeRemovalTask(Landmarks.ReefFace face) {
        scoringTasks.add(new ScoringTask(
                GameAction.RemoveAlgae,
                Optional.of(face),
                Optional.empty(),
                Optional.empty()));
    }

    public void addAlgaeProcessingTask() {
        scoringTasks.add(new ScoringTask(
                GameAction.ProcessAlgae,
                Optional.empty(),
                Optional.empty(),
                Optional.empty()));
    }

    public static boolean isTaskWellFormed(ScoringTask task) {
        if (task != null) {
            return switch (task.gameAction()) {
                case ScoreCoral ->
                        task.reefFace().isPresent() && task.branch().isPresent() && task.coralLevel().isPresent();
                case RemoveAlgae -> task.reefFace().isPresent();
                case ProcessAlgae -> true;
                default -> false;
            };
        }
        return false;
    }

    public void addScoringGoalToBottomOfQueue(ScoringTask scoringTask) {
        scoringTasks.add(scoringTask);
    }

    public void advanceToNextScoringGoal() {
        scoringTasks.poll();
    }

    public int getQueueSize() {
        return scoringTasks.size();
    }

    public ScoringTask getActiveTask() {
        return scoringTasks.peek();
    }

    public void clearQueue() {
        scoringTasks.clear();
    }

    public void clearQueueIfDefault() {
        if (defaultState) {
            defaultState = false;
            clearQueue();
        }
    }

    public List<String> getTaskArray() {
        var tasks = new ArrayList<String>();
        scoringTasks.forEach((t) -> tasks.add(t.toString()));
        return tasks;
    }
}