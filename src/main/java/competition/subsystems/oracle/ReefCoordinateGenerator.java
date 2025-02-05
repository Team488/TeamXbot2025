package competition.subsystems.oracle;

import competition.electrical_contract.ElectricalContract;
import competition.subsystems.pose.Landmarks;
import competition.subsystems.pose.PoseSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;

import javax.inject.Inject;
import javax.inject.Singleton;
import java.util.HashMap;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

@Singleton
public class ReefCoordinateGenerator {

    private final HashMap<Landmarks.ReefFace, Angle> blueReefAngleMapping;
    private final ElectricalContract contract;
    private final HashMap<String, Translation2d> handTunedOffsets;

    @Inject
    public ReefCoordinateGenerator(ElectricalContract contract) {
        blueReefAngleMapping = new HashMap<>();
        handTunedOffsets = new HashMap<>();
        this.contract = contract;


        blueReefAngleMapping.put(Landmarks.ReefFace.FAR, Degrees.of(60 * 0));
        blueReefAngleMapping.put(Landmarks.ReefFace.FAR_LEFT, Degrees.of(60 * 1));
        blueReefAngleMapping.put(Landmarks.ReefFace.CLOSE_LEFT, Degrees.of(60 * 2));
        blueReefAngleMapping.put(Landmarks.ReefFace.CLOSE, Degrees.of(60 * 3));
        blueReefAngleMapping.put(Landmarks.ReefFace.CLOSE_RIGHT, Degrees.of(60 * 4));
        blueReefAngleMapping.put(Landmarks.ReefFace.FAR_RIGHT, Degrees.of(60 * 5));
    }

    private Angle getReefRayDirection(DriverStation.Alliance alliance, Landmarks.ReefFace reefFace) {
        var reefAngle = blueReefAngleMapping.get(reefFace);
        if (alliance == DriverStation.Alliance.Red) {
            reefAngle = reefAngle.plus(Degrees.of(180));
        }

        return reefAngle;
    }

    public Pose2d getPoseRelativeToReefCenter(
            DriverStation.Alliance alliance,
            Landmarks.ReefFace reefFace,
            Distance distanceFromCenterX,
            Distance distanceFromCenterY) {
        var reefCenter = Landmarks.BlueCenterOfReef.getTranslation();
        if (alliance == DriverStation.Alliance.Red) {
            reefCenter = PoseSubsystem.convertBlueToRed(reefCenter);
        }
        var reefRayDirection = getReefRayDirection(alliance, reefFace);

        Translation2d offsetVector = new Translation2d(distanceFromCenterX.in(Meters), distanceFromCenterY.in(Meters));
        Translation2d rotatedVector = offsetVector.rotateBy(Rotation2d.fromRadians(reefRayDirection.in(Radians)));

        return new Pose2d(reefCenter.plus(rotatedVector), Rotation2d.fromRadians(reefRayDirection.in(Radians)).rotateBy(Rotation2d.fromDegrees(180)));
    }

    public Pose2d getPoseRelativeToReefFace(
            DriverStation.Alliance alliance,
            Landmarks.ReefFace reefFace,
            Distance distanceFromFaceX,
            Distance distanceFromFaceY) {
        return getPoseRelativeToReefCenter(alliance, reefFace, distanceFromFaceX.plus(Landmarks.reefCenterToFace), distanceFromFaceY);
    }

    public Pose2d getPoseRelativeToReefFaceAndBranch(
            DriverStation.Alliance alliance,
            Landmarks.ReefFace reefFace,
            Landmarks.Branch branch,
            Distance distanceFromFaceX,
            Distance distanceFromBranchY) {
        var horizontalOffset = Landmarks.reefBranchHorizontalOffsetForBranchTypeA;
        if (branch == Landmarks.Branch.B) {
            horizontalOffset = horizontalOffset.times(-1);
        }

        return getPoseRelativeToReefFace(alliance, reefFace, distanceFromFaceX, distanceFromBranchY.plus(horizontalOffset));
    }

    public Pose2d getTypicalScoringLocationForFaceBranchLevel(
            DriverStation.Alliance alliance,
            Landmarks.ReefFace reefFace,
            Landmarks.Branch branch,
            Landmarks.CoralLevel level) {

        String key = alliance.toString() + reefFace.toString() + branch.toString() + level.toString();
        Translation2d offset = new Translation2d(0, 0);
        if (handTunedOffsets.containsKey(key)) {
            offset = handTunedOffsets.get(key);
        }

        Distance totalX = contract.getDistanceFromCenterToOuterBumperX().plus(Meters.of(offset.getX()));
        Distance totalY = Meters.zero().plus(Meters.of(offset.getY()));

        return getPoseRelativeToReefFaceAndBranch(alliance, reefFace, branch, totalX, totalY);
    }
}
