package competition.subsystems.vision;

import edu.wpi.first.wpilibj.DriverStation;
import org.kobe.xbot.JClient.XTablesClient;
import org.kobe.xbot.JClient.XTablesClientManager;
import org.kobe.xbot.Utilities.Entities.VisionCoprocessor;
import org.kobe.xbot.Utilities.Entities.XTableValues;
import org.kobe.xbot.Utilities.Logger.XTablesLogger;
import org.kobe.xbot.Utilities.VisionCoprocessorCommander;
import xbot.common.advantage.DataFrameRefreshable;
import xbot.common.command.BaseSubsystem;
import xbot.common.logging.RobotAssertionManager;
import xbot.common.properties.PropertyFactory;
import xbot.common.properties.StringProperty;

import javax.inject.Inject;
import javax.inject.Singleton;
import java.util.logging.Level;

/**
 * This is a subsystem for getting data from coprocessors not related to AprilTags (e.g. data
 * about Coral, Algae, other robots, planned paths).
 */
@Singleton
public class CoprocessorCommunicationSubsystem extends BaseSubsystem implements DataFrameRefreshable {

    final RobotAssertionManager assertionManager;

    // xtables properties
    final StringProperty xtablesTargetPose;
    final StringProperty xtablesCoordinateLocation;
    final StringProperty xtablesHeadingLocation;

    // always persisted xtables client manager instance
    private XTablesClientManager xTablesClientManager;

    private final VisionCoprocessorCommander orinVisionCoprocessorCommander;

    private boolean useBackupPointToPointForPathplanning = false;



    @Inject
    public CoprocessorCommunicationSubsystem(PropertyFactory pf, RobotAssertionManager assertionManager) {
        this.assertionManager = assertionManager;
        pf.setPrefix(this);

        xtablesTargetPose = pf.createPersistentProperty("Xtables Target Pose", "target_pose");
        xtablesCoordinateLocation = pf.createPersistentProperty("Xtables Coordinate Location", "target_waypoints");
        xtablesHeadingLocation = pf.createPersistentProperty("Xtables Heading Location", "target_heading");

        xTablesClientManager = XTablesClient.getDefaultClientAsynchronously();
        XTablesLogger.setLoggingLevel(Level.OFF);
        
        this.orinVisionCoprocessorCommander = new VisionCoprocessorCommander(VisionCoprocessor.ORIN3_STATIC);
    }

    public boolean isUseBackupPointToPointForPathplanning() {
        return useBackupPointToPointForPathplanning;
    }

    public CoprocessorCommunicationSubsystem setUseBackupPointToPointForPathplanning(boolean useBackupPointToPointForPathplanning) {
        this.useBackupPointToPointForPathplanning = useBackupPointToPointForPathplanning;
        return this;
    }

    public XTablesClientManager getXTablesManager(){
        return xTablesClientManager;
    }

    /** Returns an instance of an xtables client if it can connect, else null**/
    public XTablesClient tryGetXTablesClient(){
        return xTablesClientManager.getOrNull();
    }

    public String getXtablesCoordinateLocation(){
        return xtablesCoordinateLocation.get();
    }

    public String getXtablesHeadingLocation(){
        return xtablesHeadingLocation.get();
    }

    public String getXtablesTargetPose() {
        return xtablesTargetPose.get();
    }

    public VisionCoprocessorCommander getOrinVisionCoprocessorCommander() {
        return this.orinVisionCoprocessorCommander;
    }

    public static XTableValues.Alliance fromAlliance(DriverStation.Alliance alliance) {
        return alliance.equals(DriverStation.Alliance.Blue) ? XTableValues.Alliance.BLUE : XTableValues.Alliance.RED;
    }

}
