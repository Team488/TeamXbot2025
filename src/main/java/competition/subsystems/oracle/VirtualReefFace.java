package competition.subsystems.oracle;

import competition.subsystems.pose.Landmarks;

import java.util.HashMap;

public class VirtualReefFace {

    private HashMap<ReefFaceScoringLocation, CoralState> scoringLocations;

    private final Landmarks.ReefFace face;
    private final int aprilTagId;
    private Landmarks.ReefAlgae algaePosition;

    public VirtualReefFace(Landmarks.ReefFace face, int aprilTagId, Landmarks.ReefAlgae algaePosition) {
        this.face = face;
        this.aprilTagId = aprilTagId;
        this.algaePosition = algaePosition;

        scoringLocations = new HashMap<>();
        initializeScoringLocations();
    }

    private void initializeScoringLocations() {
        initializeBranch(Landmarks.Branch.A);
        initializeBranch(Landmarks.Branch.B);
    }

    private void initializeBranch(Landmarks.Branch branch) {
        scoringLocations.put(
                new ReefFaceScoringLocation(branch, Landmarks.CoralLevel.ONE), CoralState.Absent);
        scoringLocations.put(
                new ReefFaceScoringLocation(branch, Landmarks.CoralLevel.TWO), CoralState.Absent);
        scoringLocations.put(
                new ReefFaceScoringLocation(branch, Landmarks.CoralLevel.THREE), CoralState.Absent);
        scoringLocations.put(
                new ReefFaceScoringLocation(branch, Landmarks.CoralLevel.FOUR), CoralState.Absent);
    }

    public Landmarks.ReefFace getFace() {
        return face;
    }

    public int getAprilTagId() {
        return aprilTagId;
    }

    public Landmarks.ReefAlgae getAlgaePosition() {
        return algaePosition;
    }

    public void setAlgaePosition(Landmarks.ReefAlgae algaePosition) {
        this.algaePosition = algaePosition;
    }

    public void setCoralState(ReefFaceScoringLocation location, CoralState state) {
        scoringLocations.put(location, state);
    }

    public void markAllPositionsAs(CoralState state) {
        scoringLocations.replaceAll((k, v) -> state);
    }

    private boolean doesAlgaeImpactLevel(Landmarks.CoralLevel level) {
        // ReefAlgae.LOW affects levels 2 and 3
        // ReefAlgage.HIGH affects level 3 only.

        if (algaePosition == null) {
            return false;
        }

        if (algaePosition == Landmarks.ReefAlgae.LOW) {
            return level == Landmarks.CoralLevel.TWO || level == Landmarks.CoralLevel.THREE;
        } else if (algaePosition == Landmarks.ReefAlgae.HIGH) {
            return level == Landmarks.CoralLevel.THREE;
        } else {
            return false;
        }
    }

    public int getCountOfLocationsMatchingFilters(CoralState coralStateToCheck, boolean considerAlgaeImpact, Landmarks.CoralLevel... levels) {
        int openLocationCount = 0;

        for (Landmarks.CoralLevel level : levels) {
            for (Landmarks.Branch branch : Landmarks.Branch.values()) {
                ReefFaceScoringLocation location = new ReefFaceScoringLocation(branch, level);
                CoralState state = scoringLocations.get(location);

                if (state == coralStateToCheck) {
                    if (coralStateToCheck == CoralState.Absent
                    && considerAlgaeImpact
                    && doesAlgaeImpactLevel(level)) {
                        continue;
                    }
                    openLocationCount++;
                }
            }
        }

        return openLocationCount;
    }
}
