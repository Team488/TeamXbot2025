package competition.subsystems.oracle;

import competition.subsystems.coral_scorer.CoralScorerSubsystem;
import competition.subsystems.oracle.contracts.CoralCollectionInfoSource;
import competition.subsystems.pose.Landmarks;
import competition.subsystems.pose.PoseSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import xbot.common.command.BaseSubsystem;
import xbot.common.properties.DoubleProperty;
import xbot.common.properties.PropertyFactory;
import xbot.common.trajectory.XbotSwervePoint;

import javax.inject.Inject;
import javax.inject.Singleton;

import java.util.List;

import static competition.subsystems.oracle.OracleSubsystem.PrimaryActivity.CollectCoral;
import static competition.subsystems.oracle.OracleSubsystem.PrimaryActivity.ScoreCoral;
import static edu.wpi.first.units.Units.Meters;

@Singleton
public class OracleSubsystem extends BaseSubsystem {

    public enum PrimaryActivity {
        CollectCoral,
        ScoreCoral
    }

    public enum ScoringSubstage {
        Travel,
        Approach,
        Scoring
    }

    final PoseSubsystem pose;
    final ReefCoordinateGenerator reefCoordinateGenerator;

    final ReefRoutingCircle blueReefRoutingCircle;
    final ReefRoutingCircle redReefRoutingCircle;

    Distance reefCollisionRadius = Meters.of(1.6);
    Distance reefRoutingRadius = Meters.of(2.0);

    final CoralCollectionInfoSource coralInfoSource;

    // Always set to start scoring, since we should always start a match with a coral preloaded in
    // autonomous.
    private PrimaryActivity currentActivity = ScoreCoral;
    private ScoringSubstage currentScoringSubstage = ScoringSubstage.Travel;

    private boolean firstRunInNewGoal;
    private boolean reevaluationRequested;

    private int instructionNumber;
    private OracleDriveAdvice currentDriveAdvice;
    private OracleSuperstructureAdvice currentSuperstructureAdvice;

    private Pose2d goalPose;


    final DoubleProperty elevatorActivationDistance;
    final DoubleProperty scoringActivationDistance;

    @Inject
    public OracleSubsystem(PoseSubsystem pose, CoralCollectionInfoSource coralInfoSource, PropertyFactory pf) {
        this.pose = pose;
        this.coralInfoSource = coralInfoSource;
        pf.setPrefix(this);

        elevatorActivationDistance = pf.createPersistentProperty("ElevatorActivationDistance", 1);
        scoringActivationDistance = pf.createPersistentProperty("ScoringActivationDistance", 0.05);

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

    private void setCurrentDriveAdvice(OracleDriveAdvice advice) {
        this.currentDriveAdvice = advice;
    }

    public OracleDriveAdvice getDriveAdvice() {
        return this.currentDriveAdvice;
    }

    private int getNextInstructionNumber() {
        return this.instructionNumber++;
    }

    public void requestReevaluation() {
        reevaluationRequested = true;
    }



    public OracleSuperstructureAdvice getSuperstructureAdvice() {
        return this.currentSuperstructureAdvice;
    }

    // Collection is very straightforward; immediately lower the elevator and arm to the collection level
    // and run the intake until you get something.

    boolean firstRunInScoringSubstage = true;

    // However, scoring has some substages.
    // -In the travel stage, we approach the goal but keep the elevator low.
    // -In the approach stage, we raise the elevator and arm to scoring position
    // -In the scoring stage, we run the scorer until we've confidently scored
    private void evaluateScoringSubstage() {
        switch (currentScoringSubstage) {
            case Travel:
                if (firstRunInScoringSubstage) {
                    setSuperstructureAdvice(Landmarks.CoralLevel.COLLECTING, CoralScorerSubsystem.CoralScorerState.STOPPED);
                }

                // Check if we're close enough to the goal to start scoring
                if (pose.getCurrentPose2d().getTranslation().getDistance(goalPose.getTranslation()) < elevatorActivationDistance.get()) {
                    currentScoringSubstage = ScoringSubstage.Approach;
                    firstRunInScoringSubstage = true;
                }
                break;
            case Approach:
                if (firstRunInScoringSubstage) {
                    setSuperstructureAdvice(Landmarks.CoralLevel.FOUR, CoralScorerSubsystem.CoralScorerState.STOPPED);
                }

                // Check if we're at the scoring position
                if (pose.getCurrentPose2d().getTranslation().getDistance(goalPose.getTranslation()) < elevatorActivationDistance.get()) {
                    currentScoringSubstage = ScoringSubstage.Scoring;
                    firstRunInScoringSubstage = true;
                }
                break;
            case Scoring:
                if (firstRunInScoringSubstage) {
                    setSuperstructureAdvice(Landmarks.CoralLevel.FOUR, CoralScorerSubsystem.CoralScorerState.SCORING);
                }

                // Check if we've scored
                if (coralInfoSource.confidentlyHasCoral()) {
                    currentScoringSubstage = ScoringSubstage.Travel;
                    firstRunInScoringSubstage = true;
                }
                break;
            default:
                // How did you get here?
                // When in doubt, make the mechanism safe.
                setSuperstructureAdvice(Landmarks.CoralLevel.COLLECTING, CoralScorerSubsystem.CoralScorerState.STOPPED);
                currentScoringSubstage = ScoringSubstage.Travel;
                firstRunInScoringSubstage = true;
                break;
        }
    }

    private void setSuperstructureAdvice(Landmarks.CoralLevel coralLevelToAchieve, CoralScorerSubsystem.CoralScorerState desiredScorerState) {
        var newSuperstructureAdvice = new OracleSuperstructureAdvice();
        newSuperstructureAdvice.instructionNumber = getNextInstructionNumber();
        newSuperstructureAdvice.coralLevelToAchieve = coralLevelToAchieve;
        newSuperstructureAdvice.desiredScorerState = desiredScorerState;
        setSuperstructureAdvice(newSuperstructureAdvice);
    }

    public void setSuperstructureAdvice(OracleSuperstructureAdvice advice) {
        this.currentSuperstructureAdvice = advice;
    }


    @Override
    public void periodic() {
        // TODO: refactor the common elements out of collecting and scoring
        switch (currentActivity) {
            case CollectCoral:
                // Check for first run or reevaluation
                if (firstRunInNewGoal || reevaluationRequested) {
                    var newDriveAdvice = new OracleDriveAdvice();
                    newDriveAdvice.instructionNumber = getNextInstructionNumber();
                    newDriveAdvice.path = getRecommendedCoralPickupTrajectory();
                    goalPose = newDriveAdvice.path.get(newDriveAdvice.path.size() - 1).keyPose;

                    setCurrentDriveAdvice(newDriveAdvice);

                    var newSuperstructureAdvice = new OracleSuperstructureAdvice();
                    newSuperstructureAdvice.instructionNumber = getNextInstructionNumber();
                    newSuperstructureAdvice.coralLevelToAchieve = Landmarks.CoralLevel.COLLECTING;
                    newSuperstructureAdvice.desiredScorerState = CoralScorerSubsystem.CoralScorerState.INTAKING;
                    setSuperstructureAdvice(newSuperstructureAdvice);

                    firstRunInNewGoal=false;
                    reevaluationRequested=false;
                }

                // Check if it's time to switch activities
                if (coralInfoSource.confidentlyHasCoral()) {
                    currentActivity = ScoreCoral;
                    firstRunInNewGoal = true;
                }
                break;
            case ScoreCoral:
                // Check for first run or reevaluation
                if (firstRunInNewGoal || reevaluationRequested) {
                    currentScoringSubstage = ScoringSubstage.Travel;

                    var newDriveAdvice = new OracleDriveAdvice();
                    newDriveAdvice.instructionNumber = getNextInstructionNumber();
                    newDriveAdvice.path = getRecommendedScoringTrajectory();
                    setCurrentDriveAdvice(newDriveAdvice);
                    goalPose = newDriveAdvice.path.get(newDriveAdvice.path.size() - 1).keyPose;

                    firstRunInNewGoal=false;
                    reevaluationRequested=false;
                }

                // Check if it's time to switch activities
                if (!coralInfoSource.confidentlyHasCoral()) {
                    currentActivity = CollectCoral;
                    firstRunInNewGoal = true;
                }
                break;
            default:
                // How did you get here?
                // When in doubt, go close to the drivers?
                currentActivity = CollectCoral;
                firstRunInNewGoal = true;
                break;
        }
    }
}
