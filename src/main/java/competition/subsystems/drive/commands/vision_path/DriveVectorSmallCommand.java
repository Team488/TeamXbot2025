package competition.subsystems.drive.commands.vision_path;

import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.pose.PoseSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Time;
import xbot.common.command.BaseCommand;
import xbot.common.controls.sensors.XTimer;
import xbot.common.math.XYPair;
import xbot.common.properties.DoubleProperty;
import xbot.common.properties.PropertyFactory;

import javax.inject.Inject;

import static edu.wpi.first.units.Units.Seconds;

public class DriveVectorSmallCommand extends BaseCommand {
    private double start;
    private Time duration = Seconds.of(0.25);

    private Pose2d targetPose;

    private DriveSubsystem drive;
    private PoseSubsystem poseSubsystem;

    private boolean backwards = false;

    private DoubleProperty drivePower;

    @Inject
    public DriveVectorSmallCommand(DriveSubsystem driveSubsystem,
                                   PoseSubsystem poseSubsystem,
                                   PropertyFactory pf) {

        drive = driveSubsystem;
        this.poseSubsystem = poseSubsystem;
        this.drivePower = pf.createPersistentProperty("DrivePower", 0.5);
    }

    @Override
    public void initialize() {
        log.info("Initializing DriveVectorSmallCommand");
        this.start = XTimer.getFPGATimestamp();
    }

    /**
     * The main body of a command. Called repeatedly while the command is scheduled.
     */
    @Override
    public void execute() {
        if (this.isFinished()) {
            drive.stop(); // Drive.stop doesnt stop unless called continuously
            return;
        }
        XYPair pair = new XYPair(drivePower.get(), 0);
        if (backwards) {
            pair = pair.scale(-1);

        }
        drive.move(pair, 0);
    }


    public DriveVectorSmallCommand setTargetPose(Pose2d targetPose) {
        this.targetPose = targetPose;
        return this;
    }

    public DriveVectorSmallCommand setBackwards(boolean backwards) {
        this.backwards = backwards;
        return this;
    }

    public Time getLast() {
        return duration;
    }

    public DriveVectorSmallCommand setLast(Time last) {
        this.duration = last;
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
        return (XTimer.getFPGATimestamp() - this.start) >= this.duration.in(Seconds);
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
