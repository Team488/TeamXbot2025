package competition.simulation;

import java.util.AbstractMap.SimpleEntry;
import java.util.HashSet;
import java.util.Set;
import java.util.stream.Collectors;

import javax.inject.Inject;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import xbot.common.advantage.AKitLogger;

public class ReefSimulator {
    enum ReefFace {
        FAR, FAR_LEFT, FAR_RIGHT, CLOSE, CLOSE_LEFT, CLOSE_RIGHT
    }

    enum ReefLevel {
        LEVEL_1, LEVEL_2, LEVEL_3, LEVEL_4
    }

    final AKitLogger aKitLog;

    Set<SimpleEntry<ReefFace, ReefLevel>> reefAlgaeLocations = new HashSet<>();

    @Inject
    public ReefSimulator() {
        aKitLog = new AKitLogger("Simulator/");
        this.resetField();
    }

    public void update() {
        aKitLog.record("FieldSimulation/ReefAlgae", getAlgaePoses());
    }

    public void resetField() {
        reefAlgaeLocations.clear();
        // populate algae locations with where algae starts
        reefAlgaeLocations.add(new SimpleEntry<>(ReefFace.CLOSE, ReefLevel.LEVEL_3));
        reefAlgaeLocations.add(new SimpleEntry<>(ReefFace.CLOSE_LEFT, ReefLevel.LEVEL_2));
        reefAlgaeLocations.add(new SimpleEntry<>(ReefFace.CLOSE_RIGHT, ReefLevel.LEVEL_2));
        reefAlgaeLocations.add(new SimpleEntry<>(ReefFace.FAR_LEFT, ReefLevel.LEVEL_3));
        reefAlgaeLocations.add(new SimpleEntry<>(ReefFace.FAR_RIGHT, ReefLevel.LEVEL_3));
        reefAlgaeLocations.add(new SimpleEntry<>(ReefFace.FAR, ReefLevel.LEVEL_2));
    }

    public Pose3d[] getAlgaePoses() {
        return reefAlgaeLocations.stream()
            .map(algaeLocation -> getAlgaePose(algaeLocation.getKey(), algaeLocation.getValue()))
            .toArray(Pose3d[]::new);
    }

    public Pose3d getAlgaePose(ReefFace face, ReefLevel level) {
        // given a reef face and level, return a Pose3d for where that piece of
        // algae is stuck on the reef
        // TODO: this number isn't right, update it
        Translation2d reefCenter = new Translation2d(4.4585, 4.0132);
        Translation2d reefCenterToFar = new Translation2d(0.665, 0);
        double faceAngleDeltaDeg = 60;
        Rotation2d rotation = switch(face) {
            case FAR -> Rotation2d.fromDegrees(0);
            case FAR_LEFT -> Rotation2d.fromDegrees(faceAngleDeltaDeg);
            case FAR_RIGHT -> Rotation2d.fromDegrees(-faceAngleDeltaDeg);
            case CLOSE -> Rotation2d.fromDegrees(180);
            case CLOSE_LEFT -> Rotation2d.fromDegrees(180 - faceAngleDeltaDeg);
            case CLOSE_RIGHT -> Rotation2d.fromDegrees(180 + faceAngleDeltaDeg);
            default -> throw new IllegalArgumentException("Invalid reef face");
        };
        var algaeTranslation = reefCenterToFar.rotateBy(rotation).plus(reefCenter);
        
        var z = switch (level) {
            case LEVEL_1 -> throw new IllegalArgumentException("Algae can't be on level 1");
            case LEVEL_2 -> 0.9;
            case LEVEL_3 -> 1.31;
            case LEVEL_4 -> throw new IllegalArgumentException("Algae can't be on level 4");
        };
        return new Pose3d(new Translation3d(algaeTranslation.getX(), algaeTranslation.getY(), z), new Rotation3d());
    }
    
}
