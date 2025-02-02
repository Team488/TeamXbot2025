package competition.subsystems.oracle;

import competition.subsystems.pose.Landmarks;
import competition.subsystems.pose.PoseSubsystem;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import xbot.common.command.BaseSubsystem;
import xbot.common.trajectory.XbotSwervePoint;

import javax.inject.Inject;
import javax.inject.Singleton;

import java.util.List;

import static edu.wpi.first.units.Units.Meters;

@Singleton
public class OracleSubsystem extends BaseSubsystem {

    final PoseSubsystem pose;
    final ReefCoordinateGenerator reefCoordinateGenerator;

    final ReefRoutingCircle blueReefRoutingCircle;
    final ReefRoutingCircle redReefRoutingCircle;

    Distance reefCollisionRadius = Meters.of(1.6);
    Distance reefRoutingRadius = Meters.of(2.0);

    @Inject
    public OracleSubsystem(PoseSubsystem pose) {

        this.pose = pose;

        reefCoordinateGenerator = new ReefCoordinateGenerator();

        blueReefRoutingCircle =
                new ReefRoutingCircle(
                        Landmarks.BlueCenterOfReef.getTranslation(),
                        reefRoutingRadius.in(Meters),
                        reefCollisionRadius.in(Meters));
        redReefRoutingCircle =
                new ReefRoutingCircle(
                        PoseSubsystem.convertBlueToRed(Landmarks.BlueCenterOfReef.getTranslation()),
                        reefRoutingRadius.in(Meters),
                        reefCollisionRadius.in(Meters));

        aKitLog.record("BlueCollisionCircle", blueReefRoutingCircle.visualizeInnerCollisionCircleAsTrajectory());
        aKitLog.record("BlueRoutingCircle", blueReefRoutingCircle.visualizeOuterRoutingCircleAsTrajectory());
    }

    public List<XbotSwervePoint> getRecommendedScoringTrajectory() {
        // TODO: go to more than one location.

        var penultimateWaypoint = reefCoordinateGenerator.getPoseRelativeToReefFaceAndBranch(
                DriverStation.Alliance.Blue,
                Landmarks.ReefFace.FAR_RIGHT,
                Landmarks.Branch.A,
                Meters.of(1),
                Meters.of(0));
        var finalWaypoint = Landmarks.BlueFarRightBranchA;

        var route = blueReefRoutingCircle.generateSwervePoints(pose.getCurrentPose2d(), penultimateWaypoint);
        route.add(new XbotSwervePoint(finalWaypoint, 10));

        aKitLog.record("RecommendedRoute", XbotSwervePoint.generateTrajectory(route));

        return route;
    }

    public List<XbotSwervePoint> getRecommendedCoralPickupTrajectory() {
        // TODO: go to more than one location.
        var finalWaypoint = Landmarks.BlueLeftCoralStationMid;
        var route = blueReefRoutingCircle.generateSwervePoints(pose.getCurrentPose2d(), finalWaypoint);
        aKitLog.record("RecommendedRoute", XbotSwervePoint.generateTrajectory(route));
        return route;
    }

    @Override
    public void periodic() {
    }
}
