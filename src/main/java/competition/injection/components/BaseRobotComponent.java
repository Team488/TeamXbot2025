package competition.injection.components;

import competition.electrical_contract.ElectricalContract;
import competition.motion.TrapezoidProfileManager;
import competition.operator_interface.OperatorCommandMap;
import competition.simulation.BaseSimulator;
import competition.subsystems.SubsystemDefaultCommandMap;
import competition.subsystems.coral_arm_pivot.CoralArmPivotSubsystem;
import competition.subsystems.coral_scorer.CoralScorerSubsystem;
import competition.subsystems.elevator.ElevatorSubsystem;
import competition.subsystems.oracle.OracleSubsystem;
import competition.subsystems.oracle.ReefCoordinateGenerator;
import competition.subsystems.vision.AprilTagVisionSubsystemExtended;
import competition.subsystems.elevator_mechanism.SuperstructureMechanismSubsystem;
import competition.subsystems.vision.CoprocessorCommunicationSubsystem;
import xbot.common.injection.components.BaseComponent;
import xbot.common.injection.swerve.SwerveComponentHolder;
import xbot.common.subsystems.drive.swerve.SwerveDefaultCommandMap;

public abstract class BaseRobotComponent extends BaseComponent {
    public abstract SubsystemDefaultCommandMap subsystemDefaultCommandMap();

    public abstract OperatorCommandMap operatorCommandMap();

    public abstract SwerveDefaultCommandMap swerveDefaultCommandMap();

    public abstract SwerveComponentHolder swerveComponentHolder();

    public abstract CoprocessorCommunicationSubsystem coprocessorCommunicationSubsystem();

    public abstract AprilTagVisionSubsystemExtended aprilTagVisionSubsystemExtended();

    public abstract BaseSimulator simulator();

    public abstract CoralArmPivotSubsystem armPivotSubsystem();

    public abstract ElevatorSubsystem elevatorSubsystem();

    public abstract CoralScorerSubsystem coralScorerSubsystem();

    public abstract SuperstructureMechanismSubsystem superstructureMechanismSubsystem();

    public abstract OracleSubsystem oracleSubsystem();

    public abstract ElectricalContract electricalContract();

    public abstract ReefCoordinateGenerator reefCoordinateGenerator();

    public abstract TrapezoidProfileManager.Factory trapezoidProfileManagerFactory();
}
