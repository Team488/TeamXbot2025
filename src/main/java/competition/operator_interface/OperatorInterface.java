package competition.operator_interface;

import javax.inject.Inject;
import javax.inject.Singleton;

import xbot.common.controls.sensors.XXboxController;
import xbot.common.controls.sensors.XXboxController.XXboxControllerFactory;
import xbot.common.logging.RobotAssertionManager;
import xbot.common.properties.DoubleProperty;
import xbot.common.properties.PropertyFactory;

/**
 * This class is the glue that binds the controls on the physical operator interface to the commands and command groups
 * that allow control of the robot.
 */
@Singleton
public class OperatorInterface {
    public final XXboxController gamepad;
    public final XXboxController programmerGamepad;
    public final XXboxController algaeArmGamepad;
    public final XXboxController sysIdGamepad;

    final DoubleProperty driverDeadband;
    final DoubleProperty operatorDeadband;
    final DoubleProperty algaeArmDeadband;



    @Inject
    public OperatorInterface(XXboxControllerFactory controllerFactory, RobotAssertionManager assertionManager, PropertyFactory pf) {
        gamepad = controllerFactory.create(0);
        gamepad.setLeftInversion(false, true);
        gamepad.setRightInversion(true, true);

        programmerGamepad = controllerFactory.create(3);
        programmerGamepad.setLeftInversion(false, true);
        programmerGamepad.setRightInversion(true, true);

        algaeArmGamepad=controllerFactory.create(5);
        algaeArmGamepad.setLeftInversion(false,true);
        algaeArmGamepad.setRightInversion(true,true);

        sysIdGamepad = controllerFactory.create(6);
        sysIdGamepad.setLeftInversion(false, true);
        sysIdGamepad.setRightInversion(true, true);

        pf.setPrefix("OperatorInterface");
        driverDeadband = pf.createPersistentProperty("Driver Deadband", 0.12);
        operatorDeadband = pf.createPersistentProperty("Operator Deadband", 0.15);
        algaeArmDeadband= pf.createPersistentProperty("Algae Arm Deadband", .18);
    }

    public double getDriverGamepadTypicalDeadband() {
        return driverDeadband.get();
    }

    public double getOperatorGamepadTypicalDeadband() {
        return operatorDeadband.get();
    }
    public double getAlgaeArmGamepadTypicalDeadband(){
        return algaeArmDeadband.get();
    }
}
