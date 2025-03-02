package competition.subsystems.drive.commands;

import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.pose.PoseSubsystem;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.TimestampedBoolean;
import xbot.common.command.BaseCommand;
import xbot.common.controls.sensors.XTimer;
import xbot.common.math.XYPair;
import xbot.common.properties.DoubleProperty;
import xbot.common.properties.PropertyFactory;

import javax.inject.Inject;


public class EmergencyAutonomousCommand extends BaseCommand {

    final DriveSubsystem drive;
    double startingTime;
    DoubleProperty moveRobotX;
    DoubleProperty moveRobotY;
    DoubleProperty timeAmountToMove;


    @Inject
    public EmergencyAutonomousCommand(DriveSubsystem drive, PropertyFactory pf) {
        pf.setPrefix(this);
        this.drive = drive;
        this.addRequirements(drive);
        this.moveRobotX = pf.createPersistentProperty("Move robot X", 1);
        this.moveRobotY = pf.createPersistentProperty("Move robot Y", 0);
        this.timeAmountToMove= pf.createPersistentProperty("Seconds it moves",1);

    }

    @Override
    public void initialize() {
        startingTime = XTimer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        drive.move(new XYPair(moveRobotX.get(),moveRobotY.get()), 0);
    }

    @Override
    public boolean isFinished() {
        return XTimer.getFPGATimestamp() - startingTime > timeAmountToMove.get();
    }
}