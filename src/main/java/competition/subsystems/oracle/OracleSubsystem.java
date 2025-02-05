package competition.subsystems.oracle;

import competition.subsystems.coral_scorer.CoralScorerSubsystem;
import competition.subsystems.oracle.contracts.CoralCollectionInfoSource;
import competition.subsystems.pose.Landmarks;
import competition.subsystems.pose.PoseSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import xbot.common.command.BaseSubsystem;
import xbot.common.logging.RobotAssertionManager;
import xbot.common.properties.DoubleProperty;
import xbot.common.properties.PropertyFactory;
import xbot.common.trajectory.XbotSwervePoint;

import javax.inject.Inject;
import javax.inject.Singleton;

import java.util.List;
import java.util.Optional;

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
    final RobotAssertionManager assertionManager;
    final ReefCoordinateGenerator reefCoordinateGenerator;
    final ScoringQueue scoringQueue;

    final ReefRoutingCircle blueReefRoutingCircle;
    final ReefRoutingCircle redReefRoutingCircle;

    Distance reefCollisionRadius = Meters.of(1.6);
    Distance reefRoutingRadius = Meters.of(2.0);

    final CoralCollectionInfoSource coralInfoSource;

    // Always set to start scoring, since we should always start a match with a coral preloaded in
    // autonomous.
    private PrimaryActivity currentActivity = ScoreCoral;
    private ScoringSubstage currentScoringSubstage = ScoringSubstage.Travel;

    private boolean firstRunInPrimaryActivity = true;
    private boolean reevaluationRequested = false;

    private int instructionNumber;
    private OracleDriveAdvice currentDriveAdvice;
    private OracleSuperstructureAdvice currentSuperstructureAdvice;

    private Pose2d goalPose;


    final DoubleProperty rangeToStartMovingSuperstructureMeters;
    final DoubleProperty rangeToActivateScorerMeters;

    @Inject
    public OracleSubsystem(PoseSubsystem pose, CoralCollectionInfoSource coralInfoSource,
                           ScoringQueue scoringQueue, ReefCoordinateGenerator generator, PropertyFactory pf,
                           RobotAssertionManager assertionManager) {
        this.pose = pose;
        this.assertionManager = assertionManager;
        this.coralInfoSource = coralInfoSource;
        this.scoringQueue = scoringQueue;
        this.reefCoordinateGenerator = generator;
        pf.setPrefix(this);

        rangeToStartMovingSuperstructureMeters = pf.createPersistentProperty("RangeToStartMovingSuperstructure-m", 1);
        rangeToActivateScorerMeters = pf.createPersistentProperty("RangeToActivateScorerMeters-m", 0.05);

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

        ScoringTask activeScoringTask = scoringQueue.getActiveTask();

        var penultimateWaypoint = reefCoordinateGenerator.getPoseRelativeToReefFaceAndBranch(
                DriverStation.Alliance.Blue,
                activeScoringTask.reefFace(),
                activeScoringTask.branch(),
                Meters.of(1),
                Meters.of(0));
        var finalWaypoint = reefCoordinateGenerator.getTypicalScoringLocationForFaceBranchLevel(
                DriverStation.Alliance.Blue,
                activeScoringTask.reefFace(),
                activeScoringTask.branch(),
                activeScoringTask.coralLevel());

        var route = blueReefRoutingCircle.generateSwervePoints(pose.getCurrentPose2d(), penultimateWaypoint);
        route.add(new XbotSwervePoint(finalWaypoint, 10));

        aKitLog.record("RecommendedRoute", XbotSwervePoint.generateTrajectory(route));

        return route;
    }

    public List<XbotSwervePoint> getRecommendedCoralPickupTrajectory() {
        // TODO: go to more than one location.
        // START HERE NEXT TIME!
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
                if (isScoringSubstageInitilizationRequired()) {
                    setSuperstructureAdvice(Landmarks.CoralLevel.COLLECTING, CoralScorerSubsystem.CoralScorerState.STOPPED);
                    setScoringSubstageInitilizationFinished();
                }

                // Check if we're close enough to the goal to start scoring
                if (pose.getCurrentPose2d().getTranslation().getDistance(goalPose.getTranslation()) < rangeToStartMovingSuperstructureMeters.get()) {
                    setNextScoringSubstage(ScoringSubstage.Approach);
                }
                break;
            case Approach:
                if (isScoringSubstageInitilizationRequired()) {
                    setSuperstructureAdvice(scoringQueue.getActiveTask().coralLevel(), CoralScorerSubsystem.CoralScorerState.STOPPED);
                    setScoringSubstageInitilizationFinished();
                }

                // Check if we're at the scoring position
                if (pose.getCurrentPose2d().getTranslation().getDistance(goalPose.getTranslation()) < rangeToStartMovingSuperstructureMeters.get()) {
                    setNextScoringSubstage(ScoringSubstage.Scoring);
                }
                break;
            case Scoring:
                if (isScoringSubstageInitilizationRequired()) {
                    setSuperstructureAdvice(scoringQueue.getActiveTask().coralLevel(), CoralScorerSubsystem.CoralScorerState.SCORING);
                    setScoringSubstageInitilizationFinished();
                }

                if (coralInfoSource.confidentlyHasScoredCoral()) {
                    setNextPrimaryActivity(CollectCoral);
                    scoringQueue.advanceToNextScoringGoal();
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
        setSuperstructureAdvice(new OracleSuperstructureAdvice(getNextInstructionNumber(), coralLevelToAchieve, desiredScorerState));
    }

    public void setSuperstructureAdvice(OracleSuperstructureAdvice advice) {
        this.currentSuperstructureAdvice = advice;
    }

    private void setNextPrimaryActivity(PrimaryActivity activity) {
        this.currentActivity = activity;
        firstRunInPrimaryActivity = true;
    }

    private boolean isPrimaryActivityInitilizationRequired() {
        return firstRunInPrimaryActivity;
    }

    private void setPrimaryActivityInitializationFinished() {
        firstRunInPrimaryActivity = false;
        reevaluationRequested = false;
    }

    private void setNextScoringSubstage(ScoringSubstage substage) {
        this.currentScoringSubstage = substage;
        firstRunInScoringSubstage = true;
    }

    private boolean isScoringSubstageInitilizationRequired() {
        return firstRunInScoringSubstage;
    }

    private void setScoringSubstageInitilizationFinished() {
        firstRunInScoringSubstage = false;
    }

    public void addScoringTask(ScoringTask scoringTask) {
        this.scoringQueue.clearQueueIfDefault();
        this.scoringQueue.addScoringGoalToBottomOfQueue(scoringTask);
    }


    @Override
    public void periodic() {
        // TODO: refactor the common elements out of collecting and scoring
        switch (currentActivity) {
            case CollectCoral:
                // Check for first run or reevaluation
                if (isPrimaryActivityInitilizationRequired() || reevaluationRequested) {
                    // Command the drive
                    var newDriveAdvice = new OracleDriveAdvice(getNextInstructionNumber(), getRecommendedCoralPickupTrajectory());
                    goalPose = newDriveAdvice.path().get(newDriveAdvice.path().size() - 1).keyPose;
                    setCurrentDriveAdvice(newDriveAdvice);
                    // Command the superstructure
                    var newSuperstructureAdvice = new OracleSuperstructureAdvice(
                            getNextInstructionNumber(), Landmarks.CoralLevel.COLLECTING, CoralScorerSubsystem.CoralScorerState.INTAKING);
                    setSuperstructureAdvice(newSuperstructureAdvice);

                    setPrimaryActivityInitializationFinished();
                }

                // Check if it's time to switch activities
                if (coralInfoSource.confidentlyHasCoral()) {
                    setNextPrimaryActivity(ScoreCoral);
                }
                break;
            case ScoreCoral:
                // Need to check if we have an active scoring task. If not, we stall here until something happens.
                if (scoringQueue.getActiveTask() == null) {
                    if (scoringQueue.getQueueSize() == 0) {
                        // No active task, and no more in the queue. We have to keep checking every loop to see if somebody
                        // adds something to the queue.
                        aKitLog.record("ActiveScoringTask", "No active scoring task");
                        return;
                    } else {
                        // No active task, but there are more in the queue. Let's get the next one.
                        scoringQueue.advanceToNextScoringGoal();
                        if (scoringQueue.getActiveTask() == null) {
                            // Somehow the queue is still empty!
                            return;
                        }
                    }
                }

                aKitLog.record("ActiveScoringTask", scoringQueue.getActiveTask().toString());

                // Check for first run or reevaluation
                if (isPrimaryActivityInitilizationRequired() || reevaluationRequested) {
                    currentScoringSubstage = ScoringSubstage.Travel;
                    // Command the drive
                    var newDriveAdvice = new OracleDriveAdvice(getNextInstructionNumber(), getRecommendedScoringTrajectory());
                    setCurrentDriveAdvice(newDriveAdvice);
                    goalPose = newDriveAdvice.path().get(newDriveAdvice.path().size() - 1).keyPose;

                    setPrimaryActivityInitializationFinished();
                    setNextScoringSubstage(ScoringSubstage.Travel);
                }

                evaluateScoringSubstage();
                break;
            default:
                // How did you get here?
                // When in doubt, go close to the drivers?
                currentActivity = CollectCoral;
                firstRunInPrimaryActivity = true;
                break;
        }
    }
}
