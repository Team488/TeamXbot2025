package competition.subsystems.drive.commands.vision_path;

import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.pose.PoseSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import xbot.common.command.BaseCommand;
import xbot.common.controls.sensors.XTimer;
import xbot.common.math.XYPair;

import javax.inject.Inject;

public class DriveVectorSmall extends BaseCommand {
    private double start;
    private double last = 0.25;

    private Pose2d targetPose;

    private DriveSubsystem drive;
    private PoseSubsystem poseSubsystem;

    @Inject
    public DriveVectorSmall(DriveSubsystem driveSubsystem, PoseSubsystem poseSubsystem) {
        drive = driveSubsystem;
        this.poseSubsystem = poseSubsystem;
    }

    @Override
    public void initialize() {
        log.info("Initializing DriveVectorSmall");
        this.start = XTimer.getFPGATimestamp();
    }

    /**
     * The main body of a command. Called repeatedly while the command is scheduled.
     */
    @Override
    public void execute() {
        if(this.isFinished()) {
            drive.stop(); // Drive.stop doesnt stop unless called continuously
            return;
        }
        drive.fieldOrientedDrive(XYPair.fromPolar(targetPose.getRotation().getDegrees() - 180, 0.25),
                0, poseSubsystem.getCurrentHeading().getDegrees(), new XYPair(0, 0));
    }


    public DriveVectorSmall setTargetPose(Pose2d targetPose) {
        this.targetPose = targetPose;
        return this;
    }

    public double getLast() {
        return last;
    }

    public DriveVectorSmall setLast(double last) {
        this.last = last;
        return this;
    }

    public Pose2d getTargetPose() {
        return targetPose;
    }

    /**
     * Whether the command has finished. Once a command finishes, the scheduler will call its end()
     * method and un-schedule it.
     *
     * @return whether the command has finished.
     */
    @Override
    public boolean isFinished() {
        return (XTimer.getFPGATimestamp() - this.start) >= this.last;
    }

    /**
     * The action to take when the command ends. Called when either the command finishes normally, or
     * when it interrupted/canceled.
     *
     * @param interrupted whether the command was interrupted/canceled
     */
    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }
}
