package competition.subsystems.pose;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;

import javax.inject.Singleton;

import java.util.HashMap;

import static edu.wpi.first.units.Units.Meters;

@Singleton
public class Landmarks {

    // Auto starting positions
    public static Pose2d BlueLeftStartingLine = new Pose2d(7.56, 7.262, Rotation2d.fromDegrees(180));
    public static Pose2d BlueMidStartingLine = new Pose2d(7.56, 6.135, Rotation2d.fromDegrees(180));
    public static Pose2d BlueRightStartingLine = new Pose2d(7.56, 5.044, Rotation2d.fromDegrees(180));

    // Reef faces TODO: Find positions again if scoring positions are at different distances from reef for each level
    // Close Left Reef Face
    public static Pose2d BlueCloseLeftBranchA = new Pose2d(4.002, 5.174, Rotation2d.fromDegrees(-60));
    public static Pose2d BlueCloseLeftAlgae = new Pose2d(3.834 ,5.137, Rotation2d.fromDegrees(-60));
    public static Pose2d BlueCloseLeftBranchB = new Pose2d(3.717, 5.010, Rotation2d.fromDegrees(-60));

    // Close Reef Face
    public static Pose2d BlueCloseBranchA = new Pose2d(3.2385, 4.1699, Rotation2d.fromDegrees(0));
    public static Pose2d BlueCloseAlgae = new Pose2d(3.2385, 4.0259, Rotation2d.fromDegrees(0));
    public static Pose2d BlueCloseBranchB = new Pose2d(3.2385, 3.8412, Rotation2d.fromDegrees(0));

    // Close Right Reef Face
    public static Pose2d BlueCloseRightBranchA = new Pose2d(3.718, 3.042, Rotation2d.fromDegrees(60));
    public static Pose2d BlueCloseRightAlgae = new Pose2d(3.834, 2.915, Rotation2d.fromDegrees(60));
    public static Pose2d BlueCloseRightBranchB = new Pose2d(4.002, 2.877, Rotation2d.fromDegrees(60));

    // Far Right Reef Face
    public static Pose2d BlueFarRightBranchA = new Pose2d(4.948, 2.878, Rotation2d.fromDegrees(120));
    public static Pose2d BlueFarRightAlgae = new Pose2d(5.116, 2.915, Rotation2d.fromDegrees(120));
    public static Pose2d BlueFarRightBranchB = new Pose2d(5.233, 3.041, Rotation2d.fromDegrees(120));

    // Far Reef Face
    public static Pose2d BlueFarBranchA = new Pose2d(5.701, 3.841, Rotation2d.fromDegrees(180));
    public static Pose2d BlueFarAlgae = new Pose2d(5.701, 4.026, Rotation2d.fromDegrees(180));
    public static Pose2d BlueFarBranchB = new Pose2d(5.701, 4.170, Rotation2d.fromDegrees(180));

    // Far Left Reef Face
    public static Pose2d BlueFarLeftBranchA = new Pose2d(5.233, 5.009, Rotation2d.fromDegrees(-120));
    public static Pose2d BlueFarLeftAlgae = new Pose2d(5.116, 5.137, Rotation2d.fromDegrees(-120));
    public static Pose2d BlueFarLeftBranchB = new Pose2d(4.949, 5.175, Rotation2d.fromDegrees(-120));

    // Coral Stations
    // Left Coral Station
    public static Pose2d BlueLeftCoralStationClose = new Pose2d(0.754, 6.804, Rotation2d.fromDegrees(-54.012));
    public static Pose2d BlueLeftCoralStationMid = new Pose2d(1.093, 7.043, Rotation2d.fromDegrees(-54.012));
    public static Pose2d BlueLeftCoralStationFar = new Pose2d(1.422, 7.282, Rotation2d.fromDegrees(-54.012));

    // Right Coral Station
    public static Pose2d BlueRightCoralStationClose = new Pose2d(0.764, 1.247, Rotation2d.fromDegrees(54.012));
    public static Pose2d BlueRightCoralStationMid = new Pose2d(1.093, 1.009, Rotation2d.fromDegrees(54.012));
    public static Pose2d BlueRightCoralStationFar = new Pose2d(1.422, 0.770, Rotation2d.fromDegrees(54.012));

    public static Pose2d BlueCenterOfReef = new Pose2d(new Translation2d(4.4785, 4.0132), new Rotation2d());
    public static final Distance reefCenterToFace = Meters.of(0.665);
    public static final Distance reefBranchHorizontalOffsetForBranchTypeA = Meters.of(-0.15);

    public HashMap<String, Pose2d> namesToLocations;

    public Landmarks() {
        namesToLocations = new HashMap<>();
    }

    private void addAllLevelsForBranch(DriverStation.Alliance alliance, ReefFace face, Branch branch, Pose2d location) {
        addNamedLocation(alliance, face, branch, CoralLevel.ONE, location);
        addNamedLocation(alliance, face, branch, CoralLevel.TWO, location);
        addNamedLocation(alliance, face, branch, CoralLevel.THREE, location);
        addNamedLocation(alliance, face, branch, CoralLevel.FOUR, location);
    }

    private void addNamedLocation(DriverStation.Alliance alliance, ReefFace face, Branch branch, CoralLevel level, Pose2d location) {
        String name = alliance.toString() + face.toString() + branch.toString() + level.toString();
        namesToLocations.put(name, location);
    }

    public enum ReefFace {
        CLOSE,
        CLOSE_LEFT,
        CLOSE_RIGHT,
        FAR,
        FAR_LEFT,
        FAR_RIGHT
    }

    public enum ReefAlgae {
        HIGH,
        LOW
    }

    public enum Branch {
        A,
        B
    }

    public enum CoralLevel {
        COLLECTING,
        ONE,
        TWO,
        THREE,
        FOUR
    }

    public enum CoralStation {
        LEFT,
        RIGHT
    }

    public enum CoralStationSection {
        CLOSE,
        MID,
        FAR
    }

    public static Pose2d getBranchPose(ReefFace reefFace, Branch branch) {
        EnumsToPose enumsToPose = new EnumsToPose();
        return enumsToPose.getBranchPose(reefFace, branch);
    }

    public static Pose2d getReefFacePose(ReefFace reefFace) {
        EnumsToPose enumsToPose = new EnumsToPose();
        return enumsToPose.getReefFacePose(reefFace);
    }

    public static Pose2d getCoralStationSectionPose(CoralStation station, CoralStationSection section) {
        EnumsToPose enumsToPose = new EnumsToPose();
        return enumsToPose.getCoralStationPose(station, section);
    }

    public static ReefFace getReefFaceFromTagId(int tagId) {
        return switch (tagId) {
            case 7, 18 -> ReefFace.CLOSE;
            case 6, 19 -> ReefFace.CLOSE_LEFT;
            case 11, 20 -> ReefFace.FAR_LEFT;
            case 10, 21 -> ReefFace.FAR;
            case 9, 22 -> ReefFace.FAR_RIGHT;
            case 8, 17 -> ReefFace.CLOSE_RIGHT;
            default -> ReefFace.CLOSE; // How did you get here?
        };
    }

    public enum FieldElementType{
        REEF_FACE,
        CORAL_STATION,
        ALGAE_PROCESSOR,
        OVERHEAD_THING
    }

    public static FieldElementType getFieldElementTypeForAprilTag(int tagId) {
        return switch (tagId) {
            case 1, 2, 12, 13 -> FieldElementType.CORAL_STATION;
            case 3, 16 -> FieldElementType.ALGAE_PROCESSOR;
            case 4, 5, 14, 15 -> FieldElementType.OVERHEAD_THING;
            default -> FieldElementType.REEF_FACE;
        };
    }

    public static int getAprilTagIdForReefFace(ReefFace reefFace, DriverStation.Alliance alliance) {
        return switch (reefFace) {
            case CLOSE -> alliance == DriverStation.Alliance.Blue ? 7 : 18;
            case CLOSE_LEFT -> alliance == DriverStation.Alliance.Blue ? 6 : 19;
            case FAR_LEFT -> alliance == DriverStation.Alliance.Blue ? 11 : 20;
            case FAR -> alliance == DriverStation.Alliance.Blue ? 10 : 21;
            case FAR_RIGHT -> alliance == DriverStation.Alliance.Blue ? 9 : 22;
            case CLOSE_RIGHT -> alliance == DriverStation.Alliance.Blue ? 8 : 17;
        };
    }
}