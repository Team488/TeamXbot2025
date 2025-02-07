package competition.simulation;

import javax.inject.Inject;

import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;

import competition.subsystems.lights.LightSubsystem;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import xbot.common.advantage.AKitLogger;

public class LightsSimulator {
    final LightSubsystem lightSubsystem;
    final AKitLogger aKitLog;

    final LoggedMechanism2d lightsMechanism;
    final LoggedMechanismLigament2d lightsLigament;

    @Inject
    public LightsSimulator(LightSubsystem lightSubsystem) {
        this.lightSubsystem = lightSubsystem;
        this.aKitLog = new AKitLogger("Simulator/");

        this.lightsMechanism = new LoggedMechanism2d(1, 1);
        var root = this.lightsMechanism.getRoot("lightsRoot", 0, 0.8);
        this.lightsLigament = new LoggedMechanismLigament2d("lights", 1, 0, 10, new Color8Bit(Color.kGreen));
        root.append(lightsLigament);
    }

    public void update() {
        aKitLog.record("FieldSimulation/Lights", this.lightsMechanism);

    }
}
