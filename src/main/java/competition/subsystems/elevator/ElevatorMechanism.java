package competition.subsystems.elevator;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import javax.inject.Inject;
import javax.inject.Singleton;

import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import xbot.common.command.BaseSubsystem;

@Singleton
public class ElevatorMechanism extends BaseSubsystem {
    final LoggedMechanism2d mech2d;
    final LoggedMechanismLigament2d elevatorLigament;
    final LoggedMechanismLigament2d armLigament;

    final double elevatorLigamentBaseLength = 0.7;
    final double armLigamentBaseAngle = 145;

    public Distance elevatorHeight = Meters.zero();
    public Angle armAngle = Degrees.zero();

    @Inject
    public ElevatorMechanism() {

        this.mech2d = new LoggedMechanism2d(1, 2);
        var root = mech2d.getRoot("ElevatorRoot", 0.57, 0.05);
        this.elevatorLigament = new LoggedMechanismLigament2d("elevator", elevatorLigamentBaseLength, 90, 6, new Color8Bit(Color.kBrown));
        root.append(elevatorLigament);

        
        this.armLigament = new LoggedMechanismLigament2d("arm", 0.47, armLigamentBaseAngle, 4, new Color8Bit(Color.kRed));
        elevatorLigament.append(armLigament);

        var scorerLigament = new LoggedMechanismLigament2d("scorer", 0.6, -145, 3, new Color8Bit(Color.kBlue));
        armLigament.append(scorerLigament);
    }

    public void periodic() {
        // update mechanism based on current elevatorHeight and armAngle
        elevatorLigament.setLength(elevatorLigamentBaseLength + elevatorHeight.in(Units.Meters));
        armLigament.setAngle(armLigamentBaseAngle + armAngle.in(Degrees));

        aKitLog.record("Mech2d", mech2d);

        // Record the robot relevative positive of the Elevator so AdvantageScope can render it correctly
        // NOTE: getting the arm to rotate correctly at the end of the elevator in AdvantageScope is a bit tricky, so ignoring that for now.
        aKitLog.record("ElevatorPose", new Pose3d(0, 0, elevatorHeight.in(Units.Meters), new Rotation3d()));

    }
}
