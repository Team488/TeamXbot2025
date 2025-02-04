package competition.subsystems.oracle;

import java.util.List;

import competition.subsystems.coral_scorer.CoralScorerSubsystem;
import competition.subsystems.oracle.contracts.CoralCollectionInfoSource;
import competition.subsystems.pose.Landmarks;
import competition.subsystems.pose.PoseSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import xbot.common.advantage.AKitLogger;
import xbot.common.trajectory.XbotSwervePoint;

public class CollectionStateMachine {
    public enum State {
        INITIALIZING,
        WAITING_FOR_CORAL,
    }

    public enum Result {
        RUNNING,
        CORAL_ACQUIRED,
        FAILURE,
    }

    PoseSubsystem pose;
    AKitLogger aKitLog;
    OracleSubsystem oracle;
    CoralCollectionInfoSource coralInfoSource;
    
    State state = State.INITIALIZING;
    // Pose2d goalPose;

    public void reset() {
        state = State.INITIALIZING;
    }

    public Result run() {
        switch (state) {
            case INITIALIZING:
                // TODO: see if we already have Coral, if so just end the machine now
                // Command the drive
                var newDriveAdvice = new OracleDriveAdvice(oracle.getNextInstructionNumber(), getRecommendedCoralPickupTrajectory());
                // This isn't being used for anything right now for this state machine, commenting it out
                // goalPose = newDriveAdvice.path().get(newDriveAdvice.path().size() - 1).keyPose;
                oracle.setCurrentDriveAdvice(newDriveAdvice);
                // Command the superstructure
                var newSuperstructureAdvice = new OracleSuperstructureAdvice(
                        oracle.getNextInstructionNumber(), Landmarks.CoralLevel.COLLECTING, CoralScorerSubsystem.CoralScorerState.INTAKING);
                oracle.setSuperstructureAdvice(newSuperstructureAdvice);
                state = State.WAITING_FOR_CORAL;
                break;
            case WAITING_FOR_CORAL:
                if(coralInfoSource.confidentlyHasCoral()) {
                    return Result.CORAL_ACQUIRED;
                }
                break;
        }   
        return Result.RUNNING;
    }

    public List<XbotSwervePoint> getRecommendedCoralPickupTrajectory() {
        // TODO: go to more than one location.
        var finalWaypoint = Landmarks.BlueLeftCoralStationMid;
        var route = oracle.blueReefRoutingCircle.generateSwervePoints(pose.getCurrentPose2d(), finalWaypoint);
        aKitLog.record("RecommendedRoute", XbotSwervePoint.generateTrajectory(route));
        return route;
    }
}
