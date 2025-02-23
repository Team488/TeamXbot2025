package competition.subsystems.drive.commands;

import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.pose.PoseSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import xbot.common.command.BaseCommand;
import xbot.common.math.XYPair;
import xbot.common.subsystems.drive.control_logic.HeadingModule;

import javax.inject.Inject;

import static edu.wpi.first.units.Units.Degrees;

public class RotateToHeadingWithHeadingModule extends BaseCommand {

    final HeadingModule headingModule;
    final DriveSubsystem drive;
    final PoseSubsystem pose;
    Angle targetHeading = Degrees.of(0);

    @Inject
    public RotateToHeadingWithHeadingModule(HeadingModule.HeadingModuleFactory headingModuleFactory,
                                            DriveSubsystem drive, PoseSubsystem pose) {
        this.headingModule = headingModuleFactory.create(drive.getRotateToHeadingPid());
        this.drive = drive;
        this.pose = pose;
        this.addRequirements(drive);
    }

    public void setTargetHeading(Angle targetHeading) {
        this.targetHeading = targetHeading;
    }

    @Override
    public void initialize() {
        log.info("Initializing");
        pose.setCurrentPoseInMeters(new Pose2d(pose.getCurrentPose2d().getTranslation(), new Rotation2d(0)));
    }

    @Override
    public void execute() {
        double rotationalPower = headingModule.calculateHeadingPower(targetHeading.in(Degrees));
        drive.fieldOrientedDrive(
                new XYPair(0, 0),
                rotationalPower,
                pose.getCurrentHeading().getDegrees(),
                true
        );
    }

    @Override
    public boolean isFinished() {
        return headingModule.isOnTarget();
    }
}
