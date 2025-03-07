package competition.subsystems.drive.commands;

import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.pose.Landmarks;
import competition.subsystems.pose.PoseSubsystem;
import xbot.common.command.BaseCommand;
import xbot.common.controls.sensors.XTimer;
import xbot.common.math.XYPair;
import xbot.common.properties.DoubleProperty;
import xbot.common.properties.PropertyFactory;

import javax.inject.Inject;

public class ShoveCoralStationCommand extends BaseCommand {

    DriveSubsystem drive;
    PoseSubsystem pose;
    final DoubleProperty shovePower;
    final DoubleProperty shoveWaitTime;
    private double startTime = -Double.MAX_VALUE;

    double shoveAngleDegrees;

    @Inject
    public ShoveCoralStationCommand(DriveSubsystem drive, PoseSubsystem pose, PropertyFactory pf) {
        this.drive = drive;
        this.pose = pose;

        shovePower = pf.createPersistentProperty("ShovePower", 0.25);
        shoveWaitTime = pf.createPersistentProperty("ShoveWaitTime", 0.25);

        this.addRequirements(drive);
    }

    public void setShoveAngle(Landmarks.CoralStation coralStation) {
        if (coralStation == Landmarks.CoralStation.LEFT) {
            shoveAngleDegrees = Landmarks.BlueLeftCoralStationMid.getRotation().getDegrees();
        }
        else {
            shoveAngleDegrees = Landmarks.BlueRightCoralStationMid.getRotation().getDegrees();
        }
    }

    @Override
    public void initialize() {
        super.initialize();
        log.info("Initializing");
        pose.setPreferOdometryToVision(true);
        startTime = XTimer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        drive.fieldOrientedDrive(XYPair.fromPolar(shoveAngleDegrees, shovePower.get()), 0, pose.getCurrentHeading().getDegrees(), true);
        aKitLog.record("ShoveAngleDegrees", shoveAngleDegrees);
    }

    @Override
    public boolean isFinished() {
        return XTimer.getFPGATimestamp() - startTime > shoveWaitTime.get();
    }

    @Override
    public void end(boolean interrupted) {
        pose.setPreferOdometryToVision(false);
        drive.stop();
    }
}
