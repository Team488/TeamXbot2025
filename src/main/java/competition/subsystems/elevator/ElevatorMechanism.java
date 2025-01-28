package competition.subsystems.elevator;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import javax.inject.Inject;
import javax.inject.Singleton;

import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
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
    final LoggedMechanismLigament2d coralLigament;

    // TODO: these will be replaced by reading from the subsystem(s) when they exist and can be simulated    
    public Distance elevatorHeight = Meters.zero();
    public Angle armAngle = Degrees.zero();
    public boolean coralInScorer = true;

    // these constants define the mechanism geometry rendering at zero height and zero angle
    // all tuned by trial and error
    final double elevatorLigamentBaseLengthMeters = 0.7;
    final double armLigamentBaseAngleDegrees = 145;
    final double armLengthMeters = 0.47;
    final double scorerLengthMeters = 0.6;
    final double scorerAngleDegrees = -145;
    final double coralLengthMeters = 0.3;
    // where the base of the elevator appears on the robot, 
    final Translation2d elevatorBasePositionMeters = new Translation2d(0.57, 0.05);

    @Inject
    public ElevatorMechanism(/* TODO: Inject references to the Elevator + Arm + Scorer subsystems when they exist */ ) {
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

    public void periodic() {
        // update mechanism based on current elevatorHeight and armAngle
        elevatorLigament.setLength(elevatorLigamentBaseLengthMeters + elevatorHeight.in(Units.Meters));
        armLigament.setAngle(armLigamentBaseAngleDegrees - armAngle.in(Degrees));
        coralLigament.setLength(this.coralInScorer ? coralLengthMeters : 0.0);
        aKitLog.record("Mech2d", mech2d);

        // Record the robot relevative positive of the Elevator so AdvantageScope can render it correctly
        // NOTE: getting the arm to rotate correctly at the end of the elevator in AdvantageScope is a bit tricky, so ignoring that for now.
        aKitLog.record("ElevatorPose", new Pose3d(0, 0, elevatorHeight.in(Units.Meters), new Rotation3d()));

    }
}
