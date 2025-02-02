package competition.subsystems.oracle;

import competition.subsystems.pose.Landmarks;
import competition.subsystems.pose.PoseSubsystem;
import edu.wpi.first.units.measure.Distance;
import xbot.common.command.BaseSubsystem;
import xbot.common.trajectory.XbotSwervePoint;

import javax.inject.Inject;
import javax.inject.Singleton;

import java.util.List;

import static edu.wpi.first.units.Units.Meters;

@Singleton
public class OracleSubsystem extends BaseSubsystem {

    final PoseSubsystem pose;

    final ReefRoutingCircle blueReefRoutingCircle;
    final ReefRoutingCircle redReefRoutingCircle;

    Distance reefCollisionRadius = Meters.of(1.15);
    Distance reefRoutingRadius = Meters.of(2.5);

    @Inject
    public OracleSubsystem(PoseSubsystem pose) {

        this.pose = pose;

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
        var scoringLocation = Landmarks.BlueFarRightBranchA;
        var route = blueReefRoutingCircle.generateSwervePoints(pose.getCurrentPose2d(), scoringLocation);
        aKitLog.record("ScoringRoute", XbotSwervePoint.generateTrajectory(route));

        return route;
    }

    @Override
    public void periodic() {
    }
}
