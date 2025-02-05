package competition.subsystems.oracle.reefknowledge;

import competition.subsystems.pose.Landmarks;

public class ReefFaceTracker {

    private final Landmarks.ReefFace reefFace;

    private Landmarks.ReefAlgae algaeState;
    private boolean branchALevel1Filled;
    private boolean branchALevel2Filled;
    private boolean branchALevel3Filled;
    private boolean branchALevel4Filled;
    private boolean branchBLevel1Filled;
    private boolean branchBLevel2Filled;
    private boolean branchBLevel3Filled;
    private boolean branchBLevel4Filled;

    public ReefFaceTracker(Landmarks.ReefFace reefFace) {
        this.reefFace = reefFace;
    }

    public boolean getIsFilled(Landmarks.Branch branch, Landmarks.CoralLevel level) {
        if (branch == Landmarks.Branch.A) {
            if (level == Landmarks.CoralLevel.ONE) {
                return branchALevel1Filled;
            } else if (level == Landmarks.CoralLevel.TWO) {
                return branchALevel2Filled;
            } else if (level == Landmarks.CoralLevel.THREE) {
                return branchALevel3Filled;
            } else if (level == Landmarks.CoralLevel.FOUR) {
                return branchALevel4Filled;
            }
        } else if (branch == Landmarks.Branch.B) {
            if (level == Landmarks.CoralLevel.ONE) {
                return branchBLevel1Filled;
            } else if (level == Landmarks.CoralLevel.TWO) {
                return branchBLevel2Filled;
            } else if (level == Landmarks.CoralLevel.THREE) {
                return branchBLevel3Filled;
            } else if (level == Landmarks.CoralLevel.FOUR) {
                return branchBLevel4Filled;
            }
        }
        return false;
    }

    public void setIsFilled(Landmarks.Branch branch, Landmarks.CoralLevel level, boolean isFilled) {
        if (branch == Landmarks.Branch.A) {
            if (level == Landmarks.CoralLevel.ONE) {
                branchALevel1Filled = isFilled;
            } else if (level == Landmarks.CoralLevel.TWO) {
                branchALevel2Filled = isFilled;
            } else if (level == Landmarks.CoralLevel.THREE) {
                branchALevel3Filled = isFilled;
            } else if (level == Landmarks.CoralLevel.FOUR) {
                branchALevel4Filled = isFilled;
            }
        } else if (branch == Landmarks.Branch.B) {
            if (level == Landmarks.CoralLevel.ONE) {
                branchBLevel1Filled = isFilled;
            } else if (level == Landmarks.CoralLevel.TWO) {
                branchBLevel2Filled = isFilled;
            } else if (level == Landmarks.CoralLevel.THREE) {
                branchBLevel3Filled = isFilled;
            } else if (level == Landmarks.CoralLevel.FOUR) {
                branchBLevel4Filled = isFilled;
            }
        }
    }

    public void getNumberFilledAtLevel(Landmarks.CoralLevel level) {
        switch(level) {
            case ONE:
                return (int)branchALevel1Filled + branchBLevel1Filled;
            case TWO:
                return branchALevel2Filled + branchBLevel2Filled;
            case THREE:
                return branchALevel3Filled + branchBLevel3Filled;
            case FOUR:
                return branchALevel4Filled + branchBLevel4Filled;
        }
    }
}
