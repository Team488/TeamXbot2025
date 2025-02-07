package competition.commandgroups;

import competition.subsystems.coral_arm_pivot.commands.SetCoralArmTargetAngleCommand;
import competition.subsystems.coral_scorer.commands.IntakeUntillCoral;
import competition.subsystems.elevator.commands.SetElevatorTargetHeightCommand;
import competition.subsystems.pose.Landmarks;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import xbot.common.command.BaseCommand;

import javax.inject.Inject;
import javax.inject.Provider;

public class HumanLoadUntillCoralCollectedCommandGroup extends ParallelCommandGroup {
    Provider<SetCoralArmTargetAngleCommand> setCoralArmTargetAngleCommandProvider;
    Provider<SetElevatorTargetHeightCommand> setElevatorTargetHeightCommandProvider;
    Provider<IntakeUntillCoral> intakeUntillCoralProvider;

    @Inject
    public HumanLoadUntillCoralCollectedCommandGroup(Provider<SetCoralArmTargetAngleCommand> setCoralArmTargetAngleCommandProvider,
                                                     Provider<SetElevatorTargetHeightCommand> setElevatorTargetHeightCommandProvider,
                                                     Provider<IntakeUntillCoral> intakeUntillCoralProvider){
        var setArmtoCollect= setCoralArmTargetAngleCommandProvider.get();
        setArmtoCollect.setAngle(Landmarks.CoralLevel.COLLECTING);
        var setElevatorTargetHeight= setElevatorTargetHeightCommandProvider.get();
        setElevatorTargetHeight.setHeight(Landmarks.CoralLevel.COLLECTING);
        var setIntakeUntillCoral= intakeUntillCoralProvider.get();
        setIntakeUntillCoral.isFinished();

        this.addCommands((Command) intakeUntillCoralProvider,
                (Command) setCoralArmTargetAngleCommandProvider,
                (Command) setElevatorTargetHeightCommandProvider);

        //this.setCoralArmTargetAngleCommandProvider=setCoralArmTargetAngleCommandProvider;
        //this.setElevatorTargetHeightCommandProvider=setElevatorTargetHeightCommandProvider;
        //thisintakeUntillCoralProvider=intakeUntillCoralProvider;




    }
}
