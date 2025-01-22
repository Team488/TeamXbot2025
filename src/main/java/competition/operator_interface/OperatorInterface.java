package competition.operator_interface;

import javax.inject.Inject;
import javax.inject.Singleton;

import xbot.common.controls.sensors.XXboxController;
import xbot.common.controls.sensors.XXboxController.XXboxControllerFactory;
import xbot.common.logging.RobotAssertionManager;

/**
 * This class is the glue that binds the controls on the physical operator interface to the commands and command groups
 * that allow control of the robot.
 */
@Singleton
public class OperatorInterface {
    public XXboxController gamepad;
    public XXboxController programmerGamepad;

    @Inject
    public OperatorInterface(XXboxControllerFactory controllerFactory, RobotAssertionManager assertionManager) {
        gamepad = controllerFactory.create(0);
        gamepad.setLeftInversion(false, true);
        gamepad.setRightInversion(true, true);

        programmerGamepad = controllerFactory.create(3);
        programmerGamepad.setLeftInversion(false, true);
        programmerGamepad.setRightInversion(true, true);
    }
}
