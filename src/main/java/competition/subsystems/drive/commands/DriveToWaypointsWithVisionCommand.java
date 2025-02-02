package competition.subsystems.drive.commands;

import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.pose.PoseSubsystem;
import competition.subsystems.vision.VisionSubsystem;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.kobe.xbot.Utilities.Entities.XTableValues;
import xbot.common.logging.RobotAssertionManager;
import xbot.common.properties.PropertyFactory;
import xbot.common.properties.StringProperty;
import xbot.common.subsystems.drive.control_logic.HeadingModule;
import xbot.common.trajectory.XbotSwervePoint;
import xbot.common.subsystems.drive.SwerveSimpleTrajectoryCommand;
import org.kobe.xbot.JClient.XTablesClient;

import javax.inject.Inject;
import java.util.ArrayList;
import java.util.List;

public class DriveToWaypointsWithVisionCommand extends SwerveSimpleTrajectoryCommand {

    
    DriveSubsystem drive;
    PoseSubsystem pose;
    VisionSubsystem vision;


    @Inject
    DriveToWaypointsWithVisionCommand(PoseSubsystem pose, DriveSubsystem drive, VisionSubsystem vision,
                                      PropertyFactory pf, HeadingModule.HeadingModuleFactory headingModuleFactory,
                                      RobotAssertionManager assertionManager) {
        super(drive, pose, pf, headingModuleFactory, assertionManager);
        this.pose = pose;
        this.drive = drive;
        this.vision = vision;

    }

    @Override
    public void initialize() {
        retrieveWaypointsFromVision();
        super.initialize();
    }

    //allows for driving not in a straight line
    public void prepareToDriveWithWaypoints(Translation2d[] waypoints, Rotation2d potentialRotation){
        List<XbotSwervePoint> swervePoints = new ArrayList<>();
        for (Translation2d waypoint : waypoints){
            swervePoints.add(XbotSwervePoint.createPotentiallyFilppedXbotSwervePoint(waypoint,
                (potentialRotation != null ? potentialRotation : Rotation2d.fromDegrees(180)),this.drive.getDriveToWaypointsDurationPerPoint()));
        }

        this.logic.setKeyPoints(swervePoints);
        this.logic.setAimAtGoalDuringFinalLeg(true);
//        this.logic.setDriveBackwards(true);
        this.logic.setConstantVelocity(this.drive.getDriveToWaypointsSpeed());

        reset();
    }

    //allows for driving not in a straight line
    public void retrieveWaypointsFromVision() {
        XTablesClient xclient = this.vision.getXTablesClient();

        // both potentialy null. Will not do anything if coordinates is null, but can proceed if heading is null
//        List<XTableValues.Coordinate> coordinates = xclient.getCoordinates(this.vision.getXtablesCoordinateLocation());
        List<XTableValues.Coordinate> coordinates = xclient.getCoordinates("target_waypoints");
        if(coordinates == null){
            // fail
            log.warn("No coordinates found in vision.");
            this.prepareToDriveWithWaypoints(new Translation2d[]{},Rotation2d.fromDegrees(0));
            return;
        }
        log.info("Ingested waypoints, preparing to drive.");
        Translation2d[] waypoints = new Translation2d[coordinates.size()];
        for (int i = 0;i<coordinates.size();i++) {
            XTableValues.Coordinate coordinate = coordinates.get(i);
            waypoints[i] = new Translation2d(coordinate.getX(), coordinate.getY());
        }
        
        Double heading = xclient.getDouble(this.vision.getXtablesHeadingLocation());
        Rotation2d rotation = null;
        if(heading != null){
            rotation = Rotation2d.fromRadians(heading);
        }

        this.prepareToDriveWithWaypoints(waypoints,rotation);
    }

    @Override
    public void execute() {
        super.execute();
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }

}
