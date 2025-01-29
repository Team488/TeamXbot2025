package competition.simulation.coral_scorer;

import javax.inject.Inject;
import javax.inject.Singleton;

import competition.subsystems.coral_scorer.CoralScorerSubsystem;
import edu.wpi.first.wpilibj.MockDigitalInput;
import xbot.common.controls.actuators.mock_adapters.MockCANMotorController;

@Singleton
public class CoralScorerSimulator {
    final CoralScorerSubsystem coralScorerSubsystem;
    final MockCANMotorController coralScorerMotor;
    final MockDigitalInput coralSensor;

    @Inject
    public CoralScorerSimulator(CoralScorerSubsystem coralScorerSubsystem) {
        this.coralScorerSubsystem = coralScorerSubsystem;
        this.coralScorerMotor = (MockCANMotorController) coralScorerSubsystem.motor;
        this.coralSensor = (MockDigitalInput) coralScorerSubsystem.coralSensor;
    }

    public boolean isIntaking() {
        return coralScorerMotor.getPower() > 0;
    }

    public boolean isScoring() {
        return coralSensor.get() && coralScorerMotor.getPower() < 0;
    }

    public void simulateCoralLoad() {
        coralSensor.setValue(true);
    }

    public void simulateCoralUnload() {
        coralSensor.setValue(false);
    }
}
