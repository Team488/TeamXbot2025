package competition.simulation.reef;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

import javax.inject.Inject;
import javax.inject.Singleton;

import competition.subsystems.pose.Landmarks;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;
import xbot.common.advantage.AKitLogger;

@Singleton
public class ReefSimulator {
    enum ReefFace {
        FAR, FAR_LEFT, FAR_RIGHT, CLOSE, CLOSE_LEFT, CLOSE_RIGHT
    }

    enum ReefLevel {
        LEVEL_1, LEVEL_2, LEVEL_3, LEVEL_4
    }

    enum ReefPost {
        A, B
    }

    record ReefCoralKey(ReefFace face, ReefLevel level, ReefPost post) {
    }

    record ReefAlgaeKey(ReefFace face, ReefLevel level) {
    }

    final AKitLogger aKitLog;
    public static final Translation2d reefCenter = Landmarks.BlueCenterOfReef.getTranslation();
    public static final Translation2d reefCenterToFar = new Translation2d(Landmarks.reefCenterToFace.in(Meters), 0);
    final double faceAngleDeltaDeg = 60;

    Set<ReefAlgaeKey> reefAlgaeLocations = new HashSet<>();
    Set<ReefCoralKey> reefCoralLocations = new HashSet<>();

    @Inject
    public ReefSimulator() {
        aKitLog = new AKitLogger("Simulator/");
        this.resetField();
        //this.fillReefWithCoral();
    }

    public void update() {
        aKitLog.record("FieldSimulation/ReefCoral", getCoralPoses());
        aKitLog.record("FieldSimulation/ReefAlgae", getAlgaePoses());
    }

    public void fillReefWithCoral() {
        // for debugging positions
        // enumerate faces, levels and posts
        for (ReefFace face : ReefFace.values()) {
            for (ReefLevel level : ReefLevel.values()) {
                for (ReefPost post : ReefPost.values()) {
                    reefCoralLocations.add(new ReefCoralKey(face, level, post));
                }
            }
        }
    }

    public void scoreCoralNearestTo(Translation3d scorerPose) {
        var coralDistanceMap = new HashMap<ReefCoralKey, Distance>();
        for (ReefFace face : ReefFace.values()) {
            for (ReefLevel level : ReefLevel.values()) {
                for (ReefPost post : ReefPost.values()) {
                    var key = new ReefCoralKey(face, level, post);
                    // TODO: consider some logic to ignore locations that already have coral on them
                    // or otherwise cause the coral to drop to the floor if that happens?
                    var pose = getCoralPose(key.face, key.level, key.post);
                    var distance = Meters.of(scorerPose.getDistance(pose.getTranslation()));
                    coralDistanceMap.put(key, distance);
                }
            }
        }
        // sort coralDistanceMap by distance and find the key for the smallest distance
        var closestCoral = coralDistanceMap.entrySet().stream()
            .min(Map.Entry.comparingByValue())
            .map(Map.Entry::getKey)
            .orElseThrow(() -> new IllegalStateException("No corals found"));
        System.out.println("Found closest coral: " + closestCoral);
        reefCoralLocations.add(closestCoral);
    }

    public void resetField() {
        reefCoralLocations.clear();
        reefAlgaeLocations.clear();
        // populate algae locations with where algae starts
        reefAlgaeLocations.add(new ReefAlgaeKey(ReefFace.CLOSE, ReefLevel.LEVEL_3));
        reefAlgaeLocations.add(new ReefAlgaeKey(ReefFace.CLOSE_LEFT, ReefLevel.LEVEL_2));
        reefAlgaeLocations.add(new ReefAlgaeKey(ReefFace.CLOSE_RIGHT, ReefLevel.LEVEL_2));
        reefAlgaeLocations.add(new ReefAlgaeKey(ReefFace.FAR_LEFT, ReefLevel.LEVEL_3));
        reefAlgaeLocations.add(new ReefAlgaeKey(ReefFace.FAR_RIGHT, ReefLevel.LEVEL_3));
        reefAlgaeLocations.add(new ReefAlgaeKey(ReefFace.FAR, ReefLevel.LEVEL_2));
    }

    public Pose3d[] getCoralPoses() {
        return reefCoralLocations.stream()
                .map(coralLocation -> getCoralPose(coralLocation.face(), coralLocation.level(), coralLocation.post()))
                .toArray(Pose3d[]::new);
    }

    public Pose3d getCoralPose(ReefFace face, ReefLevel level, ReefPost post) {
        // given a reef face, level, and post, return a Pose3d for where that piece of
        // coral is stuck on the reef
        var rotation = getRotationFromFarFace(face);
        var postTranslation = switch (post) {
            case A -> new Translation2d(0, -0.15);
            case B -> new Translation2d(0.0, 0.15);
        };
        // level 4 posts are farther from the center than the others
        var level4Adjustment = switch (level) {
            case LEVEL_4 -> new Translation2d(0.1, 0);
            default -> new Translation2d();
        };
        var reefCenterToFarPost = reefCenterToFar.plus(level4Adjustment).plus(postTranslation);
        var algaeTranslation = reefCenterToFarPost.rotateBy(rotation).plus(reefCenter);

        var z = switch (level) {
            case LEVEL_1 -> 0.5;
            case LEVEL_2 -> 0.8 - 0.1;
            case LEVEL_3 -> 1.21 - 0.1;
            case LEVEL_4 -> 1.83 - 0.1;
        };

        var rotation3d = switch (level) {
            case LEVEL_4 -> new Rotation3d(0, Math.PI / 2, 0);
            case LEVEL_1 -> new Rotation3d(0.0, 0.0, rotation.getRadians());
            default -> {
                // level 2 and 3 need to be rotated to specific angles
                var base = new Rotation3d(0.0, -Degrees.of(35).in(Radians), 0.0);
                yield base.rotateBy(new Rotation3d(0.0, 0.0, rotation.getRadians()));
            }
        };

        return new Pose3d(new Translation3d(algaeTranslation.getX(), algaeTranslation.getY(), z), rotation3d);
    }

    public Pose3d[] getAlgaePoses() {
        return reefAlgaeLocations.stream()
                .map(algaeLocation -> getAlgaePose(algaeLocation.face(), algaeLocation.level()))
                .toArray(Pose3d[]::new);
    }

    public Rotation2d getRotationFromFarFace(ReefFace face) {
        Rotation2d rotation = switch (face) {
            case FAR -> Rotation2d.fromDegrees(0);
            case FAR_LEFT -> Rotation2d.fromDegrees(faceAngleDeltaDeg);
            case FAR_RIGHT -> Rotation2d.fromDegrees(-faceAngleDeltaDeg);
            case CLOSE -> Rotation2d.fromDegrees(180);
            case CLOSE_LEFT -> Rotation2d.fromDegrees(180 - faceAngleDeltaDeg);
            case CLOSE_RIGHT -> Rotation2d.fromDegrees(180 + faceAngleDeltaDeg);
            default -> throw new IllegalArgumentException("Invalid reef face");
        };
        return rotation;
    }

    public Pose3d getAlgaePose(ReefFace face, ReefLevel level) {
        // given a reef face and level, return a Pose3d for where that piece of
        // algae is stuck on the reef
        // TODO: this number isn't right, update it

        var rotation = getRotationFromFarFace(face);
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
