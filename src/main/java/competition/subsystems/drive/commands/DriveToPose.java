package competition.subsystems.drive.commands;

import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.pose.Landmarks;
import competition.subsystems.pose.PoseSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import xbot.common.command.BaseCommand;
import xbot.common.math.XYPair;
import xbot.common.subsystems.drive.control_logic.HeadingModule;

import javax.inject.Inject;

public class DriveToPose extends BaseCommand {
    DriveSubsystem drive;
    PoseSubsystem pose;
    HeadingModule headingModule;

    @Inject
    public DriveToPose(DriveSubsystem drive, PoseSubsystem pose, HeadingModule.HeadingModuleFactory headingModuleFactory) {
        this.drive = drive;
        this.pose = pose;
        this.headingModule = headingModuleFactory.create(drive.getRotateToHeadingPid());;
        this.addRequirements(drive);
    }

    @Override
    public void initialize() {
        log.info("Initializing");
    }

    @Override
    public void execute() {
        var targetPose = PoseSubsystem.convertBlueToRedIfNeeded(pose.getClosestReefFacePose());
        targetPose = PoseSubsystem.convertBlueToRedIfNeeded(Landmarks.BlueCloseRightAlgae);
        var currentPose = pose.getCurrentPose2d();

        var goal = getDriveTarget(currentPose, targetPose);

        double rotationIntent = headingModule.calculateHeadingPower(goal.getRotation());

        double dx = drive.getPositionalPid().calculate(goal.getX(), currentPose.getX());
        double dy = drive.getPositionalPid().calculate(goal.getY(), currentPose.getY());
        XYPair driveIntent = new XYPair(dx, dy);


        drive.fieldOrientedDrive(driveIntent, rotationIntent, pose.getCurrentHeading().getDegrees(), new XYPair());
        aKitLog.record("Goal Pose", goal);
    }

    public Pose2d getDriveTarget(Pose2d currentPose, Pose2d targetPose) {

        // Final line up
        var reefFaceLength = Units.inchesToMeters(36.792600);
        var maxDistanceReefLineUp = 1.5;
        var offset = currentPose.relativeTo(targetPose);
        double yDistance = Math.abs(offset.getY());
        double xDistance = Math.abs(offset.getX());
        double shiftXT =
                MathUtil.clamp(
                        (yDistance / (reefFaceLength * 2)) + ((xDistance - 0.3) / (reefFaceLength * 3)),
                        0.0,
                        1.0);
        double shiftYT = MathUtil.clamp(offset.getX() / reefFaceLength, 0.0, 1.0);
        return targetPose.transformBy(
                new Transform2d(-shiftXT * maxDistanceReefLineUp,
                        Math.copySign(shiftYT * maxDistanceReefLineUp * 1.2, offset.getY()), new Rotation2d(0)));
    }

    @Override
    public void end(boolean isInterrupted) {
        super.end(isInterrupted);
        drive.stop();
    }
}