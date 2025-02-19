package competition.injection.components;

import competition.electrical_contract.ElectricalContract;
import competition.motion.TrapezoidProfileManager;
import competition.operator_interface.OperatorCommandMap;
import competition.operator_interface.OperatorInterface;
import competition.simulation.BaseSimulator;
import competition.subsystems.SubsystemDefaultCommandMap;
import competition.subsystems.algae_arm.AlgaeArmSubsystem;
import competition.subsystems.algae_collection.AlgaeCollectionSubsystem;
import competition.subsystems.coral_arm.CoralArmSubsystem;
import competition.subsystems.coral_scorer.CoralScorerSubsystem;
import competition.subsystems.elevator.ElevatorSubsystem;
import competition.subsystems.oracle.OracleSubsystem;
import competition.subsystems.oracle.ReefCoordinateGenerator;
import competition.subsystems.pose.PoseSubsystem;
import competition.subsystems.vision.AprilTagVisionSubsystemExtended;
import competition.subsystems.elevator_mechanism.SuperstructureMechanismSubsystem;
import competition.subsystems.lights.LightSubsystem;
import competition.subsystems.vision.CoprocessorCommunicationSubsystem;
import xbot.common.injection.components.BaseComponent;
import xbot.common.injection.swerve.SwerveComponentHolder;
import xbot.common.subsystems.drive.swerve.SwerveDefaultCommandMap;
import xbot.common.subsystems.pose.BasePoseSubsystem;
import xbot.common.subsystems.pose.GameField;

public abstract class BaseRobotComponent extends BaseComponent {
    @Override
    public abstract PoseSubsystem poseSubsystem();

    public abstract SubsystemDefaultCommandMap subsystemDefaultCommandMap();

    public abstract OperatorCommandMap operatorCommandMap();

    public abstract SwerveDefaultCommandMap swerveDefaultCommandMap();

    public abstract SwerveComponentHolder swerveComponentHolder();

    public abstract CoprocessorCommunicationSubsystem coprocessorCommunicationSubsystem();

    public abstract AprilTagVisionSubsystemExtended aprilTagVisionSubsystemExtended();

    public abstract BaseSimulator simulator();

    public abstract CoralArmSubsystem armPivotSubsystem();

    public abstract ElevatorSubsystem elevatorSubsystem();

    public abstract CoralScorerSubsystem coralScorerSubsystem();

    public abstract SuperstructureMechanismSubsystem superstructureMechanismSubsystem();

    public abstract OracleSubsystem oracleSubsystem();

    public abstract ElectricalContract electricalContract();

    public abstract ReefCoordinateGenerator reefCoordinateGenerator();

    public abstract LightSubsystem lightSubsystem();

    public abstract TrapezoidProfileManager.Factory trapezoidProfileManagerFactory();

    public abstract OperatorInterface operatorInterface();

    public abstract GameField gameField();

    public abstract AlgaeArmSubsystem algaeArmSubsystem();

    public abstract AlgaeCollectionSubsystem algaeCollectionSubsystem();
}
