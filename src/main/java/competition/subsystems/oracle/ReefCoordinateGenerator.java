package competition.subsystems.oracle;

import competition.subsystems.pose.Landmarks;
import competition.subsystems.pose.PoseSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;

import java.util.HashMap;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

public class ReefCoordinateGenerator {

    private HashMap<Landmarks.ReefFace, Angle> blueReefAngleMapping;
    private HashMap<Landmarks.ReefFace, Angle> redReefAngleMapping;

    public ReefCoordinateGenerator() {
        blueReefAngleMapping = new HashMap<>();
        redReefAngleMapping = new HashMap<>();

        blueReefAngleMapping.put(Landmarks.ReefFace.FAR, Degrees.of(60 * 0));
        blueReefAngleMapping.put(Landmarks.ReefFace.FAR_LEFT, Degrees.of(60 * 1));
        blueReefAngleMapping.put(Landmarks.ReefFace.CLOSE_LEFT, Degrees.of(60 * 2));
        blueReefAngleMapping.put(Landmarks.ReefFace.CLOSE, Degrees.of(60 * 3));
        blueReefAngleMapping.put(Landmarks.ReefFace.CLOSE_RIGHT, Degrees.of(60 * 4));
        blueReefAngleMapping.put(Landmarks.ReefFace.FAR_RIGHT, Degrees.of(60 * 5));

        redReefAngleMapping.put(Landmarks.ReefFace.CLOSE, Degrees.of(60 * 0));
        redReefAngleMapping.put(Landmarks.ReefFace.CLOSE_RIGHT, Degrees.of(60 * 1));
        redReefAngleMapping.put(Landmarks.ReefFace.FAR_RIGHT, Degrees.of(60 * 2));
        redReefAngleMapping.put(Landmarks.ReefFace.FAR, Degrees.of(60 * 3));
        redReefAngleMapping.put(Landmarks.ReefFace.FAR_LEFT, Degrees.of(60 * 4));
        redReefAngleMapping.put(Landmarks.ReefFace.CLOSE_LEFT, Degrees.of(60 * 5));
    }

    public Pose2d getPoseRelativeToReefCenter(DriverStation.Alliance alliance, Landmarks.ReefFace reefFace, Distance distanceFromCenterX, Distance distanceFromCenterY) {
        var reefAngleMapping = alliance == DriverStation.Alliance.Blue ? blueReefAngleMapping : redReefAngleMapping;
        var reefCenter = Landmarks.BlueCenterOfReef.getTranslation();
        if (alliance == DriverStation.Alliance.Red) {
            reefCenter = PoseSubsystem.convertBlueToRed(reefCenter);
        }
        var reefRayDirection = reefAngleMapping.get(reefFace);

        Translation2d offsetVector = new Translation2d(distanceFromCenterX.in(Meters), distanceFromCenterY.in(Meters));
        Translation2d rotatedVector = offsetVector.rotateBy(Rotation2d.fromRadians(reefRayDirection.in(Radians)));

        return new Pose2d(reefCenter.plus(rotatedVector), Rotation2d.fromRadians(reefRayDirection.in(Radians)).rotateBy(Rotation2d.fromDegrees(180)));
    }

    public Pose2d getPoseRelativeToReefFace(DriverStation.Alliance alliance, Landmarks.ReefFace reefFace, Distance distanceFromFaceX, Distance distanceFromFaceY) {
        return getPoseRelativeToReefCenter(alliance, reefFace, distanceFromFaceX.plus(Landmarks.reefCenterToFace), distanceFromFaceY);
    }

    public Pose2d getPoseRelativeToReefFaceAndBranch(DriverStation.Alliance alliance, Landmarks.ReefFace reefFace, Landmarks.Branch branch, Distance distanceFromFaceX, Distance distanceFromBranchY) {
        var horizontalOffset = Landmarks.reefBranchHorizontalOffsetForBranchTypeA;
        if (branch == Landmarks.Branch.B) {
            horizontalOffset = horizontalOffset.times(-1);
        }

        return getPoseRelativeToReefFace(alliance, reefFace, distanceFromFaceX, distanceFromBranchY.plus(horizontalOffset));
    }
}
