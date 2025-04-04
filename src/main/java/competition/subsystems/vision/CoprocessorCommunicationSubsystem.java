package competition.subsystems.vision;

import competition.subsystems.drive.logic.AlignWithCreeperCalculator;
import competition.subsystems.drive.logic.AlignWithCreeperLogger;
import competition.Robot;
import competition.subsystems.pose.PoseSubsystem;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.logging.Level;
import javax.inject.Inject;
import javax.inject.Singleton;
import org.kobe.xbot.JClient.XTablesClient;
import org.kobe.xbot.JClient.XTablesClientManager;
import org.kobe.xbot.Utilities.Entities.VisionCoprocessor;
import org.kobe.xbot.Utilities.Entities.XTableValues;
import org.kobe.xbot.Utilities.Logger.XTablesLogger;
import org.kobe.xbot.Utilities.VisionCoprocessorCommander;
import org.kobe.xbot.Utilities.XTablesByteUtils;
import xbot.common.advantage.DataFrameRefreshable;
import xbot.common.command.BaseSubsystem;
import xbot.common.controls.sensors.XTimer;
import xbot.common.logging.RobotAssertionManager;
import xbot.common.properties.DistanceProperty;
import xbot.common.properties.DoubleProperty;
import xbot.common.properties.PropertyFactory;
import xbot.common.properties.StringProperty;
import xbot.common.subsystems.pose.BasePoseSubsystem;

import static edu.wpi.first.units.Units.Meters;

/**
 * This is a subsystem for getting data from coprocessors not related to
 * AprilTags (e.g. data about Coral, Algae, other robots, planned paths).
 */
@Singleton
public class CoprocessorCommunicationSubsystem
        extends BaseSubsystem implements DataFrameRefreshable {
    final RobotAssertionManager assertionManager;

    // xtables properties
    final StringProperty xtablesTargetPose;
    final DoubleProperty lastCoralStationConfidentTimeInterval;
    final DistanceProperty lastCoralStationConfidentDistance;
    final DoubleProperty lastBargePathConfidentTimeInterval;
    final DistanceProperty lastBargePathConfidentDistance;

    final DoubleProperty lastNearestReefBranchAPathConfidentTimeInterval;
    final DistanceProperty lastNearestReefBranchAPathConfidentDistance;
    final DoubleProperty lastNearestReefBranchBPathConfidentTimeInterval;
    final DistanceProperty lastNearestReefBranchBPathConfidentDistance;
    final StringProperty xtablesCoordinateLocation;
    final StringProperty xtablesHeadingLocation;

    // always persisted xtables client manager instance
    private XTablesClientManager xTablesClientManager;

    private final VisionCoprocessorCommander orinVisionCoprocessorCommander;

    private boolean useBackupPointToPointForPathplanning = false;

    private boolean isCoprocessorHealthy = true;

    public XTableValues.BezierCurves lastCoralStationPath;
    private Double lastCoralStationTimestamp;
    public XTableValues.BezierCurves lastBargePath;
    private Double lastBargePathTimestamp;

    public XTableValues.BezierCurves pathToNearestReefBranchACurves;
    private Double pathToNearestReefBranchATimestamp;
    public XTableValues.BezierCurves pathToNearestReefBranchBCurves;
    private Double pathToNearestReefBranchBTimestamp;



    private AlignWithCreeperLogger alignWithCreeperLogger;

    @Inject
    public CoprocessorCommunicationSubsystem(
            PropertyFactory pf, RobotAssertionManager assertionManager) {
        this.assertionManager = assertionManager;
        pf.setPrefix(this);
        lastCoralStationConfidentTimeInterval = pf.createPersistentProperty(
                "lastCoralStationConfidentTimeInterval-in-seconds", 2);
        lastCoralStationConfidentDistance = pf.createPersistentProperty(
                "lastCoralStationConfidentDistance", Meters.of(1.5));
        lastBargePathConfidentTimeInterval = pf.createPersistentProperty(
                "lastBargeConfidentTimeInterval-in-seconds", 2);
        lastBargePathConfidentDistance = pf.createPersistentProperty(
                "lastBargeConfidentDistance", Meters.of(1.5));


        lastNearestReefBranchAPathConfidentTimeInterval = pf.createPersistentProperty(
                "lastNearestReefBranchAPathConfidentTimeInterval-in-seconds", 1.5);
        lastNearestReefBranchAPathConfidentDistance = pf.createPersistentProperty(
                "lastNearestReefBranchAPathConfidentDistance", Meters.of(1.5));
        lastNearestReefBranchBPathConfidentTimeInterval = pf.createPersistentProperty(
                "lastNearestReefBranchBPathConfidentTimeInterval-in-seconds", 1.5);
        lastNearestReefBranchBPathConfidentDistance = pf.createPersistentProperty(
                "lastNearestReefBranchBPathConfidentDistance", Meters.of(1.5));
        xtablesTargetPose =
                pf.createPersistentProperty("Xtables Target Pose", "target_pose");
        xtablesCoordinateLocation = pf.createPersistentProperty(
                "Xtables Coordinate Location", "target_waypoints");
        xtablesHeadingLocation = pf.createPersistentProperty(
                "Xtables Heading Location", "target_heading");
        XTablesLogger.setLoggingLevel(Level.OFF);
        xTablesClientManager = XTablesClient.getClientAsynchronously("localhost");
        xTablesClientManager.getClientFuture().thenAccept(client
                -> client.subscribe(
                "BEZIER_PATH_TO_NEAREST_CORAL_STATION", (update) -> {
                    XTableValues.BezierCurves curves =
                            XTablesByteUtils.unpack_bezier_curves(update.getValue());
                    if (curves != null) {
                        lastCoralStationTimestamp = XTimer.getFPGATimestamp();
                        lastCoralStationPath = curves;
                    }
                }));
        xTablesClientManager.getClientFuture().thenAccept(client
                -> client.subscribe(
                "BEZIER_PATH_TO_NEAREST_BARGE", (update) -> {
                    XTableValues.BezierCurves curves =
                            XTablesByteUtils.unpack_bezier_curves(update.getValue());
                    if (curves != null) {
                        lastBargePathTimestamp = XTimer.getFPGATimestamp();
                        lastBargePath = curves;
                    }
                }));
        xTablesClientManager.getClientFuture().thenAccept(client
                -> client.subscribe(
                "BEZIER_PATH_TO_NEAREST_REEF_BRANCH_A", (update) -> {
                    XTableValues.BezierCurves curves =
                            XTablesByteUtils.unpack_bezier_curves(update.getValue());
                    if (curves != null) {
                        pathToNearestReefBranchATimestamp = XTimer.getFPGATimestamp();
                        pathToNearestReefBranchACurves = curves;
                    }
                }));
        xTablesClientManager.getClientFuture().thenAccept(client
                -> client.subscribe(
                "BEZIER_PATH_TO_NEAREST_REEF_BRANCH_B", (update) -> {
                    XTableValues.BezierCurves curves =
                            XTablesByteUtils.unpack_bezier_curves(update.getValue());
                    if (curves != null) {
                        pathToNearestReefBranchBTimestamp = XTimer.getFPGATimestamp();
                        pathToNearestReefBranchBCurves = curves;
                    }
                }));

        // If current instance is a simulation then use local host (if you are running the orin code locally!)
        if(Robot.isSimulation()) {
            this.orinVisionCoprocessorCommander =
                    new VisionCoprocessorCommander(VisionCoprocessor.LOCALHOST);
        } else {
            this.orinVisionCoprocessorCommander =
                    new VisionCoprocessorCommander(VisionCoprocessor.ORIN3_STATIC);
        }
    }

    public boolean isUseBackupPointToPointForPathplanning() {
        return useBackupPointToPointForPathplanning;
    }

    public CoprocessorCommunicationSubsystem
    setUseBackupPointToPointForPathplanning(
            boolean useBackupPointToPointForPathplanning) {
        this.useBackupPointToPointForPathplanning =
                useBackupPointToPointForPathplanning;
        return this;
    }

    public XTablesClientManager getXTablesManager() {
        return xTablesClientManager;
    }

    /**
     * Returns an instance of an xtables client if it can connect, else null
     **/
    public XTablesClient tryGetXTablesClient() {
        return xTablesClientManager.getOrNull();
    }

    public boolean isXTablesFound() {
        return xTablesClientManager != null
                && xTablesClientManager.getOrNull() != null;
    }

    public boolean isCoralStationPathConfident(BasePoseSubsystem poseSubsystem) {
        if (lastCoralStationPath == null || lastCoralStationTimestamp == null) {
            return false;
        }

        XTableValues.ControlPoint start = lastCoralStationPath.getCurves(0).getControlPoints(0);
        double timeElapsed = XTimer.getFPGATimestamp() - lastCoralStationTimestamp;
        double maxTime = lastCoralStationConfidentTimeInterval.get();

        double distance = new Translation2d(start.getX(), start.getY())
                .getDistance(poseSubsystem.getCurrentPose2d().getTranslation());
        double maxDistance = lastCoralStationConfidentDistance.get().in(Meters);

        return timeElapsed < maxTime && distance <= maxDistance;
    }
    public boolean isBargePathConfident(BasePoseSubsystem poseSubsystem) {
        if (lastBargePath == null || lastBargePathTimestamp == null) {
            return false;
        }

        XTableValues.ControlPoint start = lastBargePath.getCurves(0).getControlPoints(0);
        double timeElapsed = XTimer.getFPGATimestamp() - lastBargePathTimestamp;
        double maxTime = lastBargePathConfidentTimeInterval.get();

        double distance = new Translation2d(start.getX(), start.getY())
                .getDistance(poseSubsystem.getCurrentPose2d().getTranslation());
        double maxDistance = lastBargePathConfidentDistance.get().in(Meters);

        return timeElapsed < maxTime && distance <= maxDistance;
    }

    public boolean isNearestReefBranchAPathConfident(BasePoseSubsystem poseSubsystem) {
        if (pathToNearestReefBranchACurves == null || pathToNearestReefBranchATimestamp == null) {
            return false;
        }

        XTableValues.ControlPoint start = pathToNearestReefBranchACurves.getCurves(0).getControlPoints(0);
        double timeElapsed = XTimer.getFPGATimestamp() - pathToNearestReefBranchATimestamp;
        double maxTime = lastNearestReefBranchAPathConfidentTimeInterval.get();

        double distance = new Translation2d(start.getX(), start.getY())
                .getDistance(poseSubsystem.getCurrentPose2d().getTranslation());
        double maxDistance = lastNearestReefBranchAPathConfidentDistance.get().in(Meters);

        return timeElapsed < maxTime && distance <= maxDistance;
    }
    public boolean isNearestReefBranchBPathConfident(BasePoseSubsystem poseSubsystem) {
        if (pathToNearestReefBranchBCurves == null || pathToNearestReefBranchBTimestamp == null) {
            return false;
        }

        XTableValues.ControlPoint start = pathToNearestReefBranchBCurves.getCurves(0).getControlPoints(0);
        double timeElapsed = XTimer.getFPGATimestamp() - pathToNearestReefBranchBTimestamp;
        double maxTime = lastNearestReefBranchBPathConfidentTimeInterval.get();

        double distance = new Translation2d(start.getX(), start.getY())
                .getDistance(poseSubsystem.getCurrentPose2d().getTranslation());
        double maxDistance = lastNearestReefBranchBPathConfidentDistance.get().in(Meters);

        return timeElapsed < maxTime && distance <= maxDistance;
    }

    public XTableValues.BezierCurves getPathToNearestReefBranchACurves() {
        return pathToNearestReefBranchACurves;
    }

    public XTableValues.BezierCurves getPathToNearestReefBranchBCurves() {
        return pathToNearestReefBranchBCurves;
    }

    public XTableValues.BezierCurves getLastCoralStationPath() {
        return lastCoralStationPath;
    }
    public XTableValues.BezierCurves getLastBargePath() {
        return lastBargePath;
    }


    public String getXtablesCoordinateLocation() {
        return xtablesCoordinateLocation.get();
    }

    public String getXtablesHeadingLocation() {
        return xtablesHeadingLocation.get();
    }

    public String getXtablesTargetPose() {
        return xtablesTargetPose.get();
    }

    public VisionCoprocessorCommander getOrinVisionCoprocessorCommander() {
        return this.orinVisionCoprocessorCommander;
    }

    public static XTableValues.Alliance fromAlliance(
            DriverStation.Alliance alliance) {
        return alliance.equals(DriverStation.Alliance.Blue)
                ? XTableValues.Alliance.BLUE
                : XTableValues.Alliance.RED;
    }

    public void setCoprocessorHealthy(boolean coprocessorHealthy) {
        isCoprocessorHealthy = coprocessorHealthy;
    }

    public boolean getIsCoprocessorHealthy() {
        return isCoprocessorHealthy;
    }

    @Override
    public void periodic() {
        aKitLog.record("isCoprocessorHealthy", isCoprocessorHealthy);
        if (isXTablesFound()) {
            // This allows the Orin to choose correct paths for the current alliance when possible.
            tryGetXTablesClient().putString("TEAM",
                    DriverStation.getAlliance()
                            .orElse(DriverStation.Alliance.Blue)
                            .name());
        }

        if(alignWithCreeperLogger == null){
            alignWithCreeperLogger = new AlignWithCreeperLogger(this.xTablesClientManager, this.aKitLog);
        }

        if(!alignWithCreeperLogger.isInitalized()){
            boolean creeperInitalized = alignWithCreeperLogger.initialize();

            aKitLog.record("Creeper logger Initialized", creeperInitalized);
        }
        else{
            alignWithCreeperLogger.logAlignment();
        }




    }
}