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
    public final XXboxController driverGamepad;
    public final XXboxController operatorGamepad;
    public final XXboxController neoTrellis;
    public final XXboxController oiPanel;
    public final XXboxController superstructureGamepad;
    public final XXboxController algaeAndSysIdGamepad;

    final DoubleProperty driverDeadband;
    final DoubleProperty operatorDeadband;
    final DoubleProperty algaeArmDeadband;



    @Inject
    public OperatorInterface(XXboxControllerFactory controllerFactory, RobotAssertionManager assertionManager, PropertyFactory pf) {
        driverGamepad = controllerFactory.create(0);
        driverGamepad.setLeftInversion(false, true);
        driverGamepad.setRightInversion(true, true);

        operatorGamepad = controllerFactory.create(1);
        operatorGamepad.setLeftInversion(false, true);
        operatorGamepad.setRightInversion(false, true);

        neoTrellis = controllerFactory.create(2);
        // No axes to invert on the NeoTrellis

        oiPanel = controllerFactory.create(3);
        // No axes to invert on the OI Panel

        superstructureGamepad = controllerFactory.create(4);
        superstructureGamepad.setLeftInversion(false, true);
        superstructureGamepad.setRightInversion(false, true);

        algaeAndSysIdGamepad=controllerFactory.create(5);
        algaeAndSysIdGamepad.setLeftInversion(false, true);
        algaeAndSysIdGamepad.setRightInversion(false, true);

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
}
