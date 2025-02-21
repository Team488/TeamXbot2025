package competition.subsystems.oracle;

import competition.subsystems.pose.Landmarks;
import xbot.common.properties.PropertyFactory;
import xbot.common.properties.StringProperty;

import javax.inject.Inject;
import javax.inject.Singleton;
import javax.swing.text.html.Option;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import java.util.Queue;

@Singleton
public class ScoringQueue {

    private final Queue<ScoringTask> scoringTasks;
    private boolean defaultState = true;

    final StringProperty task1;
    final StringProperty task2;
    final StringProperty task3;
    final StringProperty task4;
    final StringProperty task5;

    final HashMap<Integer, StringProperty> taskProperties;

    final String noTask = "No task";

    @Inject
    public ScoringQueue(PropertyFactory pf) {
        this.scoringTasks = new LinkedList<>();

        pf.setPrefix("ScoringQueue");
        task1 = pf.createPersistentProperty("Task1", noTask);
        task2 = pf.createPersistentProperty("Task2", noTask);
        task3 = pf.createPersistentProperty("Task3", noTask);
        task4 = pf.createPersistentProperty("Task4", noTask);
        task5 = pf.createPersistentProperty("Task5", noTask);

        taskProperties = new HashMap<>();
        taskProperties.put(1, task1);
        taskProperties.put(2, task2);
        taskProperties.put(3, task3);
        taskProperties.put(4, task4);
        taskProperties.put(5, task5);

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
        scoringTasks.forEach((t) -> tasks.add(t.toString().replace("Optional", "")));
        return tasks;
    }

    public void renderTasksToProperties() {
        var scoringArray = getTaskArray();

        for (int i= 0; i < Math.min(5, scoringArray.size()); i++) {
            var property = taskProperties.get(i+1);
            property.set(scoringArray.get(i));
        }
    }
}