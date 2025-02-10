package competition.subsystems.oracle;

import competition.subsystems.pose.Landmarks;
import edu.wpi.first.wpilibj.DriverStation;

import java.util.HashMap;

public class VirtualReef {

    HashMap<Landmarks.ReefFace, VirtualReefFace> faces;
    DriverStation.Alliance alliance;

    public VirtualReef(DriverStation.Alliance alliance) {
        faces = new HashMap<>();
        this.alliance = alliance;
        initializeFaces(alliance);
    }

    private void initializeFaces(DriverStation.Alliance alliance) {
        for (Landmarks.ReefFace face : Landmarks.allReefFaces) {
            initializeFace(face, alliance);
        }
    }

    private void initializeFace(Landmarks.ReefFace face, DriverStation.Alliance alliance) {
        faces.put(face, new VirtualReefFace(
                face,
                Landmarks.getAprilTagForAllianceReefFace(alliance, face),
                Landmarks.getAlgaePositionForReefFace(face)));
    }

    public int getCountOfLocationsMatchingFilters(CoralState coralStateToCheck, boolean considerAlgaeImpact, Landmarks.CoralLevel... levels) {
        int count = 0;
        for (VirtualReefFace face : faces.values()) {
            count += face.getCountOfLocationsMatchingFilters(coralStateToCheck, considerAlgaeImpact, levels);
        }
        return count;
    }

    public void setCoralState(Landmarks.ReefFace face, ReefFaceScoringLocation location, CoralState state) {
        VirtualReefFace reefFace = faces.get(face);
        if (reefFace != null  && location != null && state != null) {
            reefFace.setCoralState(location, state);
        }
    }

    public void markFaceAsUnavailable(Landmarks.ReefFace face) {
        VirtualReefFace reefFace = faces.get(face);
        if (reefFace != null) {
            reefFace.markAllPositionsAs(CoralState.Unavailable);
        }
    }
}