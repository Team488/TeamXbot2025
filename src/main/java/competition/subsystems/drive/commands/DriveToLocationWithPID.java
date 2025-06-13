package competition.subsystems.drive.commands;

import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.pose.PoseSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import xbot.common.command.BaseCommand;
import xbot.common.math.XYPair;

import javax.inject.Inject;

public class DriveToLocationWithPID extends BaseCommand {

    final DriveSubsystem drive;
    final PoseSubsystem pose;
    Translation2d locationTarget = new Translation2d(0, 0);

    @Inject
    public DriveToLocationWithPID(DriveSubsystem drive, PoseSubsystem pose) {
        this.drive = drive;
        this.pose = pose;
        this.addRequirements(drive);
    }

    public void setLocationTarget(Translation2d location) {
        this.locationTarget = location;
    }

    @Override
    public void initialize() {
        log.info("Initializing");
        // Reset rotation as well just in case
        pose.setCurrentPoseInMeters(new Pose2d(0, 0, new Rotation2d(0)));
    }

    @Override
    public void execute() {
        Translation2d power = drive.getPowerToAchieveFieldPosition(
                pose.getCurrentPose2d().getTranslation(),
                locationTarget
        );

        drive.fieldOrientedDrive(
                new XYPair(power.getX(), power.getY()),
                0,
                pose.getCurrentHeading().getDegrees(),
                true
        );
    }

    @Override
    public boolean isFinished() {
        return drive.getPositionalPid().isOnTarget();
    }
}
