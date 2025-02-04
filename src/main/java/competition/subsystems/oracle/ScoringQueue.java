package competition.subsystems.oracle;

import competition.subsystems.pose.Landmarks;

import javax.inject.Inject;
import javax.inject.Singleton;
import java.util.LinkedList;
import java.util.Queue;

@Singleton
public class ScoringQueue {

    private final Queue<ScoringTask> scoringTasks;

    @Inject
    public ScoringQueue() {
        this.scoringTasks = new LinkedList<>();

        // TODO: set this up based on some commands invoked by the operator (before auto, or during the match)
        // for now, cheating in some tasks.
        scoringTasks.add(new ScoringTask(Landmarks.ReefFace.CLOSE_LEFT, Landmarks.Branch.A, Landmarks.CoralLevel.FOUR));
        scoringTasks.add(new ScoringTask(Landmarks.ReefFace.CLOSE_LEFT, Landmarks.Branch.B, Landmarks.CoralLevel.FOUR));
        scoringTasks.add(new ScoringTask(Landmarks.ReefFace.CLOSE_LEFT, Landmarks.Branch.A, Landmarks.CoralLevel.THREE));
        scoringTasks.add(new ScoringTask(Landmarks.ReefFace.CLOSE_LEFT, Landmarks.Branch.B, Landmarks.CoralLevel.THREE));

        scoringTasks.add(new ScoringTask(Landmarks.ReefFace.FAR_LEFT, Landmarks.Branch.A, Landmarks.CoralLevel.FOUR));
        scoringTasks.add(new ScoringTask(Landmarks.ReefFace.FAR_LEFT, Landmarks.Branch.B, Landmarks.CoralLevel.FOUR));
        scoringTasks.add(new ScoringTask(Landmarks.ReefFace.FAR_LEFT, Landmarks.Branch.A, Landmarks.CoralLevel.THREE));
        scoringTasks.add(new ScoringTask(Landmarks.ReefFace.FAR_LEFT, Landmarks.Branch.B, Landmarks.CoralLevel.THREE));

        scoringTasks.add(new ScoringTask(Landmarks.ReefFace.CLOSE, Landmarks.Branch.A, Landmarks.CoralLevel.FOUR));
        scoringTasks.add(new ScoringTask(Landmarks.ReefFace.CLOSE, Landmarks.Branch.B, Landmarks.CoralLevel.FOUR));
        scoringTasks.add(new ScoringTask(Landmarks.ReefFace.CLOSE, Landmarks.Branch.A, Landmarks.CoralLevel.THREE));
        scoringTasks.add(new ScoringTask(Landmarks.ReefFace.CLOSE, Landmarks.Branch.B, Landmarks.CoralLevel.THREE));
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
}