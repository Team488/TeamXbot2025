package competition.subsystems.elevator;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Inches;

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
    final LoggedMechanismLigament2d coralArmLigament;
    final LoggedMechanismLigament2d coralPresenceLigament;
    final LoggedMechanismLigament2d algaeArmLigament;
    

    // this is the state of the mechanism
    Distance elevatorHeight = Meters.zero();
    Angle coralArmAngle = Degrees.zero();
    boolean coralInScorer = false;
    Angle algaeArmAngle = Degrees.zero();

    // these constants define the mechanism geometry rendering at zero height and zero angle
    // all tuned by trial and error
    public final double elevatorLigamentBaseLengthMeters = 0.7;
    final double coralArmLigamentBaseAngleDegrees = 145;
    final double coralArmLengthMeters = 0.47;
    final double scorerLengthMeters = 0.6;
    final double scorerAngleDegrees = -145;
    final double coralLengthMeters = 0.3;
    // where the base of the elevator appears on the robot, 
    final Translation2d elevatorBasePositionMeters = new Translation2d(0.57, 0.05);

    // where the base of the algae arm appears on the robot
    final Translation2d algaeArmBasePositionMeters = new Translation2d(0.65, Inches.of(28).in(Meters));
    final double algaeArmLengthMeters = Inches.of(18.0).in(Meters);
    final double algaeArmBaseAngleDegrees = -90;


    public SuperstructureMechanism() {
        this.mech2d = new LoggedMechanism2d(1, 2);

        // coral system (elevator, arm, scorer)
        var elevatorRoot = mech2d.getRoot("ElevatorRoot", elevatorBasePositionMeters.getX(), elevatorBasePositionMeters.getY());
        this.elevatorLigament = new LoggedMechanismLigament2d("elevator", elevatorLigamentBaseLengthMeters, 90, 6, new Color8Bit(Color.kBrown));
        elevatorRoot.append(elevatorLigament);

        this.coralArmLigament = new LoggedMechanismLigament2d("coralArm", coralArmLengthMeters, coralArmLigamentBaseAngleDegrees, 4, new Color8Bit(Color.kRed));
        elevatorLigament.append(coralArmLigament);

        this.coralPresenceLigament = new LoggedMechanismLigament2d("coralPresence", coralLengthMeters, scorerAngleDegrees, 10, new Color8Bit(Color.kBeige));
        coralArmLigament.append(coralPresenceLigament);
        var scorerLigament = new LoggedMechanismLigament2d("scorer", scorerLengthMeters, scorerAngleDegrees, 3, new Color8Bit(Color.kBlue));
        coralArmLigament.append(scorerLigament);

        // algae arm
        var algaeArmRoot = mech2d.getRoot("AlgaeArmRoot", algaeArmBasePositionMeters.getX(), algaeArmBasePositionMeters.getY());
        this.algaeArmLigament = new LoggedMechanismLigament2d("algaeArm", algaeArmLengthMeters, algaeArmBaseAngleDegrees, 4, new Color8Bit(Color.kPurple));
        algaeArmRoot.append(algaeArmLigament);
    }

    public void setElevatorHeight(Distance height) {
        elevatorHeight = height;
    }

    public void setCoralArmAngle(Angle angle) {
        coralArmAngle = angle;
    }

    public void setCoralInScorer(boolean inScorer) {
        coralInScorer = inScorer;
    }

    public LoggedMechanism2d getMechanism() {
        // update mechanism based on current elevatorHeight and armAngle
        elevatorLigament.setLength(elevatorLigamentBaseLengthMeters + elevatorHeight.in(Units.Meters));
        coralArmLigament.setAngle(coralArmLigamentBaseAngleDegrees - coralArmAngle.in(Degrees));
        // fake showing/hiding coral by changing the segment width        
        coralPresenceLigament.setLineWeight(coralInScorer ? 10 : 0.001);

        algaeArmLigament.setAngle(algaeArmBaseAngleDegrees + algaeArmAngle.in(Degrees));

        return mech2d;
    }

    public void setAlgaeArmAngle(Angle armAngle) {
        algaeArmAngle = armAngle;
    }
}
