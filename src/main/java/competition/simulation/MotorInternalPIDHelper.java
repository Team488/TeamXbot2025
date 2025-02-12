package competition.simulation;

import static edu.wpi.first.units.Units.Rotations;

import xbot.common.controls.actuators.mock_adapters.MockCANMotorController;
import xbot.common.math.PIDManager;

public class MotorInternalPIDHelper {
    public static void updateInternalPID(MockCANMotorController motor, PIDManager pidManager) {
        updateInternalPID(motor, pidManager, 0.0);
    }

    public static void updateInternalPID(MockCANMotorController motor, PIDManager pidManager, double gravityFeedForward) {
        // based on the motor state, potentially run internal PID if need be
        if (motor.getControlMode() == MockCANMotorController.ControlMode.Position) {
            // run a simple pid to mimic the internal pid of the motor controller
            var targetPosition = motor.getTargetPosition();
            var currentPosition = motor.getPosition();
            var output = pidManager.calculate(targetPosition.in(Rotations), currentPosition.in(Rotations))
                    + gravityFeedForward;
            motor.setPower(output);
        } else {
            pidManager.reset();
        }
    }
}
