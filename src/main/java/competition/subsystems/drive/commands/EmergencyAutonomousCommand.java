package competition.subsystems.drive.commands;

import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.pose.PoseSubsystem;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.TimestampedBoolean;
import xbot.common.command.BaseCommand;
import xbot.common.controls.sensors.XTimer;
import xbot.common.math.XYPair;

import javax.inject.Inject;


public class EmergencyAutonomousCommand extends BaseCommand {

    DriveSubsystem drive;
    double startingTime;

    @Inject
    public EmergencyAutonomousCommand(DriveSubsystem drive) {
        this.drive = drive;
    }

    @Override
    public void initialize() {
        startingTime = XTimer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        drive.move(new XYPair(1,0), 0);
    }

    @Override
    public boolean isFinished() {
        return XTimer.getFPGATimestamp() - startingTime > 1;
    }
}