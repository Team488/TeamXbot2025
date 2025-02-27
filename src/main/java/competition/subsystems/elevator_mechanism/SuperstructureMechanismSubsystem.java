package competition.subsystems.elevator_mechanism;

import javax.inject.Inject;

import competition.subsystems.coral_arm.CoralArmSubsystem;
import competition.subsystems.coral_scorer.CoralScorerSubsystem;
import competition.subsystems.elevator.SuperstructureMechanism;
import competition.subsystems.elevator.ElevatorSubsystem;
import xbot.common.command.BaseSubsystem;

/*
 * Responsible for rendering a Mech2d object representing the elevator-arm-scorer system as perceived by the robot code.
 */
public class SuperstructureMechanismSubsystem extends BaseSubsystem {
    final ElevatorSubsystem elevatorSubsystem;
    final CoralArmSubsystem armPivotSubsystem;
    final CoralScorerSubsystem coralScorerSubsystem;

    final SuperstructureMechanism superstructureMechanism;

    @Inject
    public SuperstructureMechanismSubsystem(ElevatorSubsystem elevatorSubsystem,
                                            CoralArmSubsystem armPivotSubsystem, CoralScorerSubsystem coralScorerSubsystem) {
        this.elevatorSubsystem = elevatorSubsystem;
        this.armPivotSubsystem = armPivotSubsystem;
        this.coralScorerSubsystem = coralScorerSubsystem;

        superstructureMechanism = new SuperstructureMechanism();
    }

    @Override
    public void periodic() {
        superstructureMechanism.setElevatorHeight(elevatorSubsystem.getCurrentValue());
        superstructureMechanism.setCoralArmAngle(armPivotSubsystem.getCurrentValue());
        superstructureMechanism.setCoralInScorer(coralScorerSubsystem.hasCoral());

        aKitLog.record("SuperstructureMechanism", superstructureMechanism.getMechanism());
    }
}
