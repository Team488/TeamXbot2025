package competition.subsystems.oracle;

import competition.subsystems.pose.Landmarks;

public record ScoringTask (Landmarks.ReefFace reefFace, Landmarks.Branch branch, Landmarks.CoralLevel coralLevel){
    public String toString() {
        return reefFace.toString() + branch.toString() + coralLevel.toString();
    }
}
