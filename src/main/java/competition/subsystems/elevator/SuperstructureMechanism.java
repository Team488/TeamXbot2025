package competition.subsystems.elevator;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class SuperstructureMechanism {
    final LoggedMechanism2d mech2d;
    final LoggedMechanismLigament2d elevatorLigament;
    final LoggedMechanismLigament2d armLigament;
    final LoggedMechanismLigament2d coralLigament;

    Distance elevatorHeight = Meters.zero();
    Angle armAngle = Degrees.zero();
    boolean coralInScorer = false;

    // these constants define the mechanism geometry rendering at zero height and zero angle
    // all tuned by trial and error
    public final double elevatorLigamentBaseLengthMeters = 0.7;
    final double armLigamentBaseAngleDegrees = 145;
    final double armLengthMeters = 0.47;
    final double scorerLengthMeters = 0.6;
    final double scorerAngleDegrees = -145;
    final double coralLengthMeters = 0.3;
    // where the base of the elevator appears on the robot, 
    final Translation2d elevatorBasePositionMeters = new Translation2d(0.57, 0.05);

    public SuperstructureMechanism() {
        this.mech2d = new LoggedMechanism2d(1, 2);
        var root = mech2d.getRoot("ElevatorRoot", elevatorBasePositionMeters.getX(), elevatorBasePositionMeters.getY());
        this.elevatorLigament = new LoggedMechanismLigament2d("elevator", elevatorLigamentBaseLengthMeters, 90, 6, new Color8Bit(Color.kBrown));
        root.append(elevatorLigament);

        
        this.armLigament = new LoggedMechanismLigament2d("arm", armLengthMeters, armLigamentBaseAngleDegrees, 4, new Color8Bit(Color.kRed));
        elevatorLigament.append(armLigament);

        this.coralLigament = new LoggedMechanismLigament2d("coral", coralLengthMeters, scorerAngleDegrees, 10, new Color8Bit(Color.kBeige));
        armLigament.append(coralLigament);
        var scorerLigament = new LoggedMechanismLigament2d("scorer", scorerLengthMeters, scorerAngleDegrees, 3, new Color8Bit(Color.kBlue));
        armLigament.append(scorerLigament);
    }

    public void setElevatorHeight(Distance height) {
        elevatorHeight = height;
    }

    public void setArmAngle(Angle angle) {
        armAngle = angle;
    }

    public void setCoralInScorer(boolean inScorer) {
        coralInScorer = inScorer;
    }

    public LoggedMechanism2d getMechanism() {
        // update mechanism based on current elevatorHeight and armAngle
        elevatorLigament.setLength(elevatorLigamentBaseLengthMeters + elevatorHeight.in(Units.Meters));
        armLigament.setAngle(armLigamentBaseAngleDegrees - armAngle.in(Degrees));
        // fake showing/hiding coral by changing the segment length between 0 and full length
        coralLigament.setLength(coralInScorer ? coralLengthMeters : 0.0);

        return mech2d;
    }
}
