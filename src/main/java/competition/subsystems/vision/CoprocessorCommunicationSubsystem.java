package competition.subsystems.vision;

import org.kobe.xbot.JClient.XTablesClient;
import org.kobe.xbot.JClient.XTablesClientManager;
import xbot.common.advantage.DataFrameRefreshable;
import xbot.common.command.BaseSubsystem;
import xbot.common.logging.RobotAssertionManager;
import xbot.common.properties.PropertyFactory;
import xbot.common.properties.StringProperty;

import javax.inject.Inject;
import javax.inject.Singleton;

/**
 * This is a subsystem for getting data from coprocessors not related to AprilTags (e.g. data
 * about Coral, Algae, other robots, planned paths).
 */
@Singleton
public class CoprocessorCommunicationSubsystem extends BaseSubsystem implements DataFrameRefreshable {

    final RobotAssertionManager assertionManager;

    // xtables properties
    final StringProperty xtablesCoordinateLocation;
    final StringProperty xtablesHeadingLocation;

    // always persisted xtables instance
    private XTablesClientManager xTablesClientManager;
    private WPIXTablesClient wxclient;


    @Inject
    public CoprocessorCommunicationSubsystem(PropertyFactory pf, RobotAssertionManager assertionManager) {
        this.assertionManager = assertionManager;
        pf.setPrefix(this);

        xtablesCoordinateLocation = pf.createPersistentProperty("Xtables Coordinate Location", "target_waypoints");
        xtablesHeadingLocation = pf.createPersistentProperty("Xtables Heading Location", "target_heading");

        xTablesClientManager = XTablesClient.getDefaultClientAsynchronously();
        wxclient = new WPIXTablesClient(xTablesClientManager);
    }

    public XTablesClientManager getXTablesClient(){
        // in case of any weirdness
        return xTablesClientManager;
    }
    /** Returns wrapper around XTablesClient, Adds methods to put wpi classes**/
    public WPIXTablesClient getWPIXTablesClient(){
        // in case of any weirdness
        return wxclient;
    }

    public String getXtablesCoordinateLocation(){
        return xtablesCoordinateLocation.get();
    }

    public String getXtablesHeadingLocation(){
        return xtablesHeadingLocation.get();
    }
}