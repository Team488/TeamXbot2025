package competition.subsystems.vision;

import xbot.common.advantage.DataFrameRefreshable;
import xbot.common.command.BaseSubsystem;
import xbot.common.logging.RobotAssertionManager;
import xbot.common.properties.PropertyFactory;

import javax.inject.Inject;
import javax.inject.Singleton;

/**
 * This is a subsystem for getting data from coprocessors not related to AprilTags (e.g. data
 * about Coral, Algae, other robots, planned paths).
 */
@Singleton
public class CoprocessorCommunicationSubsystem extends BaseSubsystem implements DataFrameRefreshable {

    final RobotAssertionManager assertionManager;

    @Inject
    public CoprocessorCommunicationSubsystem(PropertyFactory pf, RobotAssertionManager assertionManager) {
        this.assertionManager = assertionManager;

        pf.setPrefix(this);
    }
}