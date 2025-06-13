package competition.subsystems.oracle;

import competition.subsystems.pose.Landmarks;

import java.util.Optional;

/**
 * A game action that the oracle can attempt to service.
 * @param gameAction e.g. scoring coral, removing algae, processing algae
 * @param reefFace Which reef face (relevant for coral and removing algae)
 * @param branch Which branch (only relevant for coral, empty otherwise)
 * @param coralLevel Which level (only relevant for coral, empty otherwise)
 */
public record ScoringTask (
        GameAction gameAction,
        Optional<Landmarks.ReefFace> reefFace,
        Optional<Landmarks.Branch> branch,
        Optional<Landmarks.CoralLevel> coralLevel){

    public String toString() {
        return gameAction.toString() + reefFace.toString() + branch.toString() + coralLevel.toString();
    }
}
