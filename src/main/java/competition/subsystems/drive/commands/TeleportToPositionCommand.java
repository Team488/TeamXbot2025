package competition.subsystems.drive.commands;

import javax.inject.Inject;

import org.kobe.xbot.JClient.XTablesClient;
import org.kobe.xbot.JClient.XTablesClientManager;

import competition.operator_interface.OperatorInterface;
import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.pose.Landmarks;
import competition.subsystems.pose.PoseSubsystem;
import competition.subsystems.vision.CoprocessorCommunicationSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Units;
import xbot.common.command.BaseCommand;
import xbot.common.math.MathUtils;
import xbot.common.subsystems.pose.BasePoseSubsystem;

public class TeleportToPositionCommand extends BaseCommand {

    final DriveSubsystem driveSubsystem;
    final CoprocessorCommunicationSubsystem coprocessorComs;
    final PoseSubsystem poseSubsystem;
    private boolean hasRun = false;

    @Inject
    public TeleportToPositionCommand(PoseSubsystem poseSubsystem, DriveSubsystem driveSubsystem, CoprocessorCommunicationSubsystem coprocessorComs) {
        this.poseSubsystem = poseSubsystem;
        this.driveSubsystem = driveSubsystem;
        this.coprocessorComs = coprocessorComs;
    }

    @Override
    public void initialize() {
        log.info("Initializing");

    }

    @Override
    public void execute() {
        XTablesClient client = coprocessorComs.tryGetXTablesClient();
        if(client == null){
            log.warn("Xtables client is null, not setting command!");
            cancel();
            return;
        }

        double defaultX = PoseSubsystem.fieldXMidpointInMeters.in(Units.Meter);
        double defaultY = PoseSubsystem.fieldYMidpointInMeters.in(Units.Meter);
        double defaultYaw = 0;

        try{
            Double networkX = client.getDouble("setXPosition");
            if(networkX != null){
                defaultX = networkX;
            }
            
            Double networkY = client.getDouble("setYPosition");
            if(networkY != null){
                defaultY = networkY;
            }
    
            Double networkYaw = client.getDouble("setYawRotation");
            if(networkYaw != null){
                defaultYaw = networkYaw;
            }
        }
        catch(IllegalArgumentException illegalArgumentException){
            log.warn("Invalid types recieved from xclient");
        }
        

        log.info(String.format("Set Position X(m): %f Y(m): %f Yaw(Degrees): %f", defaultX,defaultY,defaultYaw));

        this.poseSubsystem.setCurrentPoseInMeters(new Pose2d(defaultX, defaultY, Rotation2d.fromDegrees(defaultYaw)));
        this.hasRun = true;


    }

    @Override
    public boolean isFinished(){
        return this.hasRun;
    }
}
