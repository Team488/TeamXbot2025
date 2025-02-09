package competition.subsystems.vision;

import org.kobe.xbot.JClient.CachedSubscriber;
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
    final StringProperty xtablesWayPointLocation;
    final StringProperty xtablesHeadingLocation;

    // always persisted xtables client manager instance
    private XTablesClientManager xTablesClientManager;
    private CachedSubscriber wayPointSubscriber = null;
    private CachedSubscriber headingSubscriber = null;


    @Inject
    public CoprocessorCommunicationSubsystem(PropertyFactory pf, RobotAssertionManager assertionManager) {
        this.assertionManager = assertionManager;
        pf.setPrefix(this);

        xtablesWayPointLocation = pf.createPersistentProperty("Xtables Coordinate Location", "target_waypoints");
        xtablesHeadingLocation = pf.createPersistentProperty("Xtables Heading Location", "target_heading");

        xTablesClientManager = XTablesClient.getDefaultClientAsynchronously();



    }

    public CachedSubscriber getWayPointSubscriber() {
        if(wayPointSubscriber == null) {
            // try get xclient
            XTablesClient client = this.xTablesClientManager.getOrNull();
            if(client != null) {
                // try create
                wayPointSubscriber = new CachedSubscriber("target_waypoints",client);
            }
        }
        return wayPointSubscriber;
    }

    public CachedSubscriber getHeadingSubscriber() {
        if(headingSubscriber == null) {
            // try get xclient
            XTablesClient client = this.xTablesClientManager.getOrNull();
            if(client != null) {
                // try create
                headingSubscriber = new CachedSubscriber(xtablesHeadingLocation.get(),client);
            }
        }
        return headingSubscriber;
    }

    public XTablesClientManager getXTablesManager(){
        return xTablesClientManager;
    }

    /** Returns an instance of an xtables client if it can connect, else null**/
    public XTablesClient tryGetXTablesClient(){
        return xTablesClientManager.getOrNull();
    }

    public String getXtablesWayPointLocation(){
        return xtablesWayPointLocation.get();
    }

    public String getXtablesHeadingLocation(){
        return xtablesHeadingLocation.get();
    }
}