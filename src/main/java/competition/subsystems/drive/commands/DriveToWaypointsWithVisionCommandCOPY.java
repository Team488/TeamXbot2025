package competition.subsystems.drive.commands;

import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.pose.PoseSubsystem;
import competition.subsystems.vision.CoprocessorCommunicationSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.kobe.xbot.JClient.CachedSubscriber;
import org.kobe.xbot.Utilities.Entities.XTableValues;
import xbot.common.logging.RobotAssertionManager;
import xbot.common.properties.PropertyFactory;
import xbot.common.subsystems.drive.SwerveSimpleTrajectoryCommand;
import xbot.common.subsystems.drive.SwerveSimpleTrajectoryMode;
import xbot.common.subsystems.drive.control_logic.HeadingModule;
import xbot.common.trajectory.XbotSwervePoint;

import javax.inject.Inject;
import java.util.ArrayList;
import java.util.List;

public class DriveToWaypointsWithVisionCommandCOPY extends SwerveSimpleTrajectoryCommand {


    DriveSubsystem drive;
    PoseSubsystem pose;
    CoprocessorCommunicationSubsystem coprocessorComms;



    @Inject
    DriveToWaypointsWithVisionCommandCOPY(PoseSubsystem pose, DriveSubsystem drive, CoprocessorCommunicationSubsystem coprocessorComms,
                                          PropertyFactory pf, HeadingModule.HeadingModuleFactory headingModuleFactory,
                                          RobotAssertionManager assertionManager) {
        super(drive, pose, pf, headingModuleFactory, assertionManager);
        this.pose = pose;
        this.drive = drive;
        this.coprocessorComms = coprocessorComms;

    }

    @Override
    public void initialize() {
        if(retrieveWaypointsFromVision()){
            log.info("Retrived waypoints from vision");
            super.initialize();
        }
        else{
            log.warn("No retrived waypoints from vision");
            super.cancel();
        }
    }

    //allows for driving not in a straight line
    public void prepareToDriveWithWaypoints(Translation2d[] waypoints, Rotation2d potentialRotation) {
        List<XbotSwervePoint> swervePoints = new ArrayList<>();
        
        for (Translation2d waypoint : waypoints) {
            swervePoints.add(new XbotSwervePoint(waypoint, (potentialRotation != null ? potentialRotation : Rotation2d.kZero),
                             this.drive.getDriveToWaypointsDurationPerPoint().get()));
        }

        this.logic.setKeyPoints(swervePoints);
        // this.logic.setAimAtGoalDuringFinalLeg(true);
        this.logic.setConstantVelocity(this.drive.getDriveToWaypointsSpeed().get());
        this.logic.setVelocityMode(SwerveSimpleTrajectoryMode.ConstantVelocity);

        // keep as reminder: if we change the command to poll continously we will need to reset everytime we update keypoints
        // likely not going to happen, this command will stay as is and a new command will take movement vectors instead 
        // reset();
    }

    //allows for driving not in a straight line
    public boolean retrieveWaypointsFromVision() {
        CachedSubscriber wayPointSubscriber = this.coprocessorComms.getWayPointSubscriber();
        // both potentialy null. Will not do anything if coordinates is null, but can proceed if heading is null
        if(wayPointSubscriber != null){
            List<XTableValues.Coordinate> coordinates = wayPointSubscriber.getAsCoordinates(null);
            if (coordinates == null) {
                // fail
                log.warn("No coordinates found in vision.");
                return false;
            }
            log.info("Ingested waypoints, preparing to drive.");
            Translation2d[] waypoints = new Translation2d[coordinates.size()];
            for (int i = 0; i < coordinates.size(); i++) {
                XTableValues.Coordinate coordinate = coordinates.get(i);
                waypoints[i] = new Translation2d(coordinate.getX(), coordinate.getY());
            }
            this.aKitLog.record("Used waypoints", waypoints);

            CachedSubscriber headingSubscriber = this.coprocessorComms.getHeadingSubscriber();
            double heading = headingSubscriber.getAsDouble(0d);
            Rotation2d rotation = Rotation2d.fromRadians(heading);
            this.aKitLog.record("Used heading", rotation);

            this.prepareToDriveWithWaypoints(waypoints, rotation);
            return true;
        }
        return false;
    }

    @Override
    public void execute() {
        super.execute();
    }

}
