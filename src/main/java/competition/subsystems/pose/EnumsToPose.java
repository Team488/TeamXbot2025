package competition.subsystems.pose;

import edu.wpi.first.math.geometry.Pose2d;

import java.util.HashMap;

public class EnumsToPose {
    HashMap<String, Pose2d> pose2dHashMap = new HashMap<>();

    public EnumsToPose() {
        addToMap(Landmarks.ReefFace.CLOSE, Landmarks.Branch.A, Landmarks.BlueCloseBranchA);
        addToMap(Landmarks.ReefFace.CLOSE, Landmarks.Branch.B, Landmarks.BlueCloseBranchB);
        addToMap(Landmarks.ReefFace.CLOSE_LEFT, Landmarks.Branch.A, Landmarks.BlueCloseLeftBranchA);
        addToMap(Landmarks.ReefFace.CLOSE_LEFT, Landmarks.Branch.B, Landmarks.BlueCloseLeftBranchB);
        addToMap(Landmarks.ReefFace.CLOSE_RIGHT, Landmarks.Branch.A, Landmarks.BlueCloseRightBranchA);
        addToMap(Landmarks.ReefFace.CLOSE_RIGHT, Landmarks.Branch.B, Landmarks.BlueCloseRightBranchB);
        addToMap(Landmarks.ReefFace.FAR, Landmarks.Branch.A, Landmarks.BlueFarBranchA);
        addToMap(Landmarks.ReefFace.FAR, Landmarks.Branch.B, Landmarks.BlueFarBranchB);
        addToMap(Landmarks.ReefFace.FAR_LEFT, Landmarks.Branch.A, Landmarks.BlueFarLeftBranchA);
        addToMap(Landmarks.ReefFace.FAR_LEFT, Landmarks.Branch.B, Landmarks.BlueFarLeftBranchB);
        addToMap(Landmarks.ReefFace.FAR_RIGHT, Landmarks.Branch.A, Landmarks.BlueFarRightBranchA);
        addToMap(Landmarks.ReefFace.FAR_RIGHT, Landmarks.Branch.B, Landmarks.BlueFarRightBranchB);

        addToMap(Landmarks.ReefFace.CLOSE, Landmarks.BlueCloseAlgae);
        addToMap(Landmarks.ReefFace.CLOSE_LEFT, Landmarks.BlueCloseLeftAlgae);
        addToMap(Landmarks.ReefFace.CLOSE_RIGHT, Landmarks.BlueCloseRightAlgae);
        addToMap(Landmarks.ReefFace.FAR, Landmarks.BlueFarAlgae);
        addToMap(Landmarks.ReefFace.FAR_LEFT, Landmarks.BlueFarLeftAlgae);
        addToMap(Landmarks.ReefFace.FAR_RIGHT, Landmarks.BlueFarRightAlgae);
    }

    public void addToMap(Landmarks.ReefFace reefFace, Landmarks.Branch branch, Pose2d pose) {
        pose2dHashMap.put(reefFace.toString() + branch.toString(), pose);
    }

    public void addToMap(Landmarks.ReefFace reefFace, Pose2d pose) {
        pose2dHashMap.put(reefFace.toString(), pose);
    }

    public Pose2d getBranchPose(Landmarks.ReefFace reefFace, Landmarks.Branch branch) {
        return pose2dHashMap.get(reefFace.toString() + branch.toString());
    }

    public Pose2d getReefFacePose(Landmarks.ReefFace reefFace) {
        return pose2dHashMap.get(reefFace.toString());
    }
}
