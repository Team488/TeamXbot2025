package competition.commandgroups;

import competition.subsystems.coral_arm.CoralArmSubsystem;
import competition.subsystems.coral_arm.commands.SetCoralArmTargetAngleCommand;
import competition.subsystems.elevator.ElevatorSubsystem;
import competition.subsystems.elevator.commands.SetElevatorTargetHeightCommand;
import competition.subsystems.pose.Landmarks;
import xbot.common.command.BaseCommand;
import xbot.common.controls.sensors.XTimer;

import javax.inject.Inject;

public class TestingScoringLevelFour extends BaseCommand {
    SetElevatorTargetHeightCommand setElevatorTargetHeightCommand;
    SetCoralArmTargetAngleCommand setCoralArmTargetAngleCommand;
    ElevatorSubsystem elevatorSubsystem;
    CoralArmSubsystem coralArmSubsystem;
    private double startTime = 0;


    @Inject
    public TestingScoringLevelFour(SetElevatorTargetHeightCommand setElevatorTargetHeightCommand,
                                   SetCoralArmTargetAngleCommand setCoralArmTargetAngleCommand,
                                   ElevatorSubsystem elevatorSubsystem, CoralArmSubsystem coralArmSubsystem) {
        this.setElevatorTargetHeightCommand = setElevatorTargetHeightCommand;
        this.setCoralArmTargetAngleCommand = setCoralArmTargetAngleCommand;
        this.elevatorSubsystem = elevatorSubsystem;
        this.coralArmSubsystem = coralArmSubsystem;
    }
    @Override
    public void initialize() {
        elevatorSubsystem.setTargetHeight(Landmarks.CoralLevel.FOUR);
        coralArmSubsystem.setTargetAngle(Landmarks.CoralLevel.FOUR);

        startTime = XTimer.getFPGATimestamp();
    }

    @Override
    public boolean isFinished() {
        return elevatorSubsystem.isMaintainerAtGoal() && coralArmSubsystem.isMaintainerAtGoal();
    }

    @Override
    public void end(boolean interrupted) {
        var duration = XTimer.getFPGATimestamp() - startTime;
        aKitLog.record("setting L4 duration", duration);
        log.info("setting L4 duration: ", duration);
    }

}
