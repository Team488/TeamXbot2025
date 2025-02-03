package competition.subsystems.oracle;

import competition.subsystems.pose.Landmarks;

import javax.inject.Inject;
import javax.inject.Singleton;
import java.util.LinkedList;
import java.util.Queue;

@Singleton
public class ScoringQueue {

    private final Queue<ScoringTask> scoringTasks;
    private ScoringTask activeTask;

    @Inject
    public ScoringQueue() {
        this.scoringTasks = new LinkedList<>();

        // for now, cheating in some tasks. Later this should be set up during autonomous or during the match.
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





        scoringTasks.add(new ScoringTask(Landmarks.ReefFace.CLOSE_LEFT, Landmarks.Branch.A, Landmarks.CoralLevel.TWO));
        scoringTasks.add(new ScoringTask(Landmarks.ReefFace.CLOSE_LEFT, Landmarks.Branch.B, Landmarks.CoralLevel.TWO));
    }

    public void addScoringGoalToBottomOfQueue(ScoringTask scoringTask) {
        scoringTasks.add(scoringTask);
    }

    public void advanceToNextScoringGoal() {
        if (scoringTasks.isEmpty()) {
            this.activeTask = null;
            return;
        }
        this.activeTask = scoringTasks.poll();
    }

    public int getQueueSize() {
        return scoringTasks.size();
    }

    public ScoringTask getActiveTask() {
        return activeTask;
    }

    public void clearQueue() {
        scoringTasks.clear();
    }
}