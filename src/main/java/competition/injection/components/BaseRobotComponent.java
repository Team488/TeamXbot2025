package competition.injection.components;

import competition.operator_interface.OperatorCommandMap;
import competition.simulation.BaseSimulator;
import competition.subsystems.SubsystemDefaultCommandMap;
import competition.subsystems.arm_pivot.ArmPivotSubsystem;
import competition.subsystems.coral_scorer.CoralScorerSubsystem;
import competition.subsystems.elevator.ElevatorSubsystem;
import competition.subsystems.vision.VisionSubsystem;
import xbot.common.injection.components.BaseComponent;
import xbot.common.injection.swerve.SwerveComponentHolder;
import xbot.common.subsystems.drive.swerve.SwerveDefaultCommandMap;
import xbot.common.subsystems.vision.AprilTagVisionSubsystem;

public abstract class BaseRobotComponent extends BaseComponent {
    public abstract SubsystemDefaultCommandMap subsystemDefaultCommandMap();

    public abstract OperatorCommandMap operatorCommandMap();

    public abstract SwerveDefaultCommandMap swerveDefaultCommandMap();

    public abstract SwerveComponentHolder swerveComponentHolder();

    public abstract VisionSubsystem visionSubsystem();

    public abstract AprilTagVisionSubsystem aprilTagVisionSubsystem();

    public abstract BaseSimulator simulator();

    public abstract ArmPivotSubsystem armPivotSubsystem();

    public abstract ElevatorSubsystem elevatorSubsystem();

    public abstract CoralScorerSubsystem coralScorerSubsystem();

}
