package competition.subsystems.oracle.reefknowledge;

import competition.subsystems.pose.Landmarks;

public class ReefTracker {

    // 6 faces, 2 branches, 4 levels
    // first 12 elements represent the 12 1st level positions.
    // second 12 elements represent the 12 2nd level positions.
    // third 12 elements represent the 12 3rd level positions.
    // fourth 12 elements represent the 12 4th level positions.
    // an entry of 0 means nothing is there.
    // an entry of 1 means a coral is scored there.
    final int[] reefBranches = new int[6*2*4];

    public ReefTracker() {
    }

    public void resetReefTrackerToGameStart() {
        for (int i = 0; i < reefBranches.length; i++) {
            reefBranches[i] = 0;
        }
    }

    public int getNumberOfCoralScoredOnLevel(Landmarks.CoralLevel level) {
        int startingIndex = 0;
        int endingIndex = 11;

        if (level == Landmarks.CoralLevel.ONE) {
            // set range to 0-11;
            startingIndex = 0;
            endingIndex = 11;
        }
        else if (level == Landmarks.CoralLevel.TWO) {
            // set range to 12-23;
            startingIndex = 12;
            endingIndex = 23;
        }
        else if (level == Landmarks.CoralLevel.THREE) {
            // set range to 24-35;
            startingIndex = 24;
            endingIndex = 35;
        }
        else if (level == Landmarks.CoralLevel.FOUR) {
            // set range to 36-47;
            startingIndex = 36;
            endingIndex = 47;
        }

        // add up all elements in the range
        int coralCount = 0;
        for (int i = startingIndex; i <= endingIndex; i++) {
            coralCount += reefBranches[i];
        }

        return coralCount;
    }

}
