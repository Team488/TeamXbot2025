package competition.subsystems.oracle;

import competition.subsystems.pose.Landmarks;

public class ScoringTask {
    public Landmarks.ReefFace reefFace;
    public Landmarks.Branch branch;
    public Landmarks.CoralLevel coralLevel;

    public ScoringTask(Landmarks.ReefFace reefFace, Landmarks.Branch branch, Landmarks.CoralLevel coralLevel) {
        this.reefFace = reefFace;
        this.branch = branch;
        this.coralLevel = coralLevel;
    }

    public String toString() {
        return reefFace.toString() + branch.toString() + coralLevel.toString();
    }
}
