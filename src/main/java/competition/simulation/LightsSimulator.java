package competition.simulation;

import javax.inject.Inject;

import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;

import competition.subsystems.lights.LightSubsystem;
import competition.subsystems.lights.LightSubsystem.LightsStateMessage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import xbot.common.advantage.AKitLogger;

public class LightsSimulator {
    final LightSubsystem lightSubsystem;
    final AKitLogger aKitLog;

    final LoggedMechanism2d lightsMechanism;
    final LoggedMechanismLigament2d lightsLigament;

    int frame;

    @Inject
    public LightsSimulator(LightSubsystem lightSubsystem) {
        this.lightSubsystem = lightSubsystem;
        this.aKitLog = new AKitLogger("Simulator/");

        this.lightsMechanism = new LoggedMechanism2d(1, 1);
        var root = this.lightsMechanism.getRoot("lightsRoot", 0.7, 0.2);
        this.lightsLigament = new LoggedMechanismLigament2d("lights", 0.5, 90, 14, new Color8Bit(Color.kBlack));
        root.append(lightsLigament);
    }

    Color8Bit getColorForSubsystemState(LightsStateMessage state) {
        if(!DriverStation.isDSAttached()) {
            return new Color8Bit(Color.kGray);
        }
        switch (state) {
            case RobotDisabled:
                return new Color8Bit(Color.kRed);
            case RobotEnabled:
                return new Color8Bit(Color.kGreen);
            case CoralPresent:
                return new Color8Bit(Color.kBlue);
            case RequestCoralFromHuman:
                return new Color8Bit(Color.kTurquoise);
            case ReadyToScore:
                return new Color8Bit(Color.kWhite);
            default:
                return new Color8Bit(Color.kBlack);
        }
    }

    public void update() {
        this.frame++;
        this.lightsLigament.setLineWeight(Math.cos(frame * Math.PI / 50) * 1 + 14);
        this.lightsLigament.setColor(getColorForSubsystemState(lightSubsystem.getState()));
        aKitLog.record("FieldSimulation/Lights", this.lightsMechanism);

    }
}
