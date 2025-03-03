package competition.subsystems.vision;

import org.kobe.xbot.JClient.XTablesClient;
import org.kobe.xbot.JClient.XTablesClientManager;
import org.kobe.xbot.Utilities.Entities.VisionCoprocessor;
import org.kobe.xbot.Utilities.Logger.XTablesLogger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

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
    private final XTablesClientManager xTablesClientManager;
    private final VisionCoprocessorCommander orinVisionCoprocessorCommander;

    private VisionCoprocessorCommander visionCoprocessorCommander;


    @Inject
    public CoprocessorCommunicationSubsystem(PropertyFactory pf, RobotAssertionManager assertionManager) {
        this.assertionManager = assertionManager;
        pf.setPrefix(this);

        xtablesTargetPose = pf.createPersistentProperty("Xtables Target Pose", "target_pose");
        xtablesCoordinateLocation = pf.createPersistentProperty("Xtables Coordinate Location", "target_waypoints");
        xtablesHeadingLocation = pf.createPersistentProperty("Xtables Heading Location", "target_heading");

        xTablesClientManager = XTablesClient.getDefaultClientAsynchronously();
//        this.orinVisionCoprocessorCommander = new VisionCoprocessorCommander("10.4.88.7"); // Connect to ORIN-3
        this.orinVisionCoprocessorCommander = new VisionCoprocessorCommander(VisionCoprocessor.LOCALHOST); // Connect to ORIN-3

        XTablesLogger.setLoggingLevel(Level.OFF);

    }

    @Override
    public void periodic(){
        super.periodic();
        this.updateXTablesInformation();

    }

    private void updateXTablesInformation(){
        XTablesClient client = this.xTablesClientManager.getOrNull();
        if(client == null){
            this.log.warn("Xtables client is returning null when trying to update xtables info!");
            return;
        }

        client.putString("TEAM", DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? "Blue" : "Red");
    }

    public XTablesClientManager getXTablesManager(){
        return xTablesClientManager;
    }

    public VisionCoprocessorCommander getOrinVisionCoprocessorCommander() {
        return orinVisionCoprocessorCommander;
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

    public VisionCoprocessorCommander getVisionCoprocessorCommander() {
        return this.visionCoprocessorCommander;
    }

}
