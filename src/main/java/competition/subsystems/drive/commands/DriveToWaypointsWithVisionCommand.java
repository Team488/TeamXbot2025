package competition.subsystems.drive.commands;

import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.pose.PoseSubsystem;
import competition.subsystems.vision.CoprocessorCommunicationSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.kobe.xbot.JClient.XTablesClient;
import org.kobe.xbot.JClient.XTablesClientManager;
import org.kobe.xbot.Utilities.Entities.XTableValues;
import xbot.common.logging.RobotAssertionManager;
import xbot.common.properties.PropertyFactory;
import xbot.common.subsystems.drive.SwerveSimpleTrajectoryCommand;
import xbot.common.subsystems.drive.control_logic.HeadingModule;
import xbot.common.trajectory.XbotSwervePoint;

import javax.inject.Inject;
import java.util.AbstractMap;
import java.util.Arrays;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;
import java.util.stream.IntStream;

public class DriveToWaypointsWithVisionCommand extends SwerveSimpleTrajectoryCommand {
    DriveSubsystem drive;
    PoseSubsystem pose;
    CoprocessorCommunicationSubsystem coprocessorComms;
    List<XTableValues.Coordinate> lastSetCoordinates;

    @Inject
    DriveToWaypointsWithVisionCommand(PoseSubsystem pose, DriveSubsystem drive, CoprocessorCommunicationSubsystem coprocessorComms,
                                      PropertyFactory pf, HeadingModule.HeadingModuleFactory headingModuleFactory,
                                      RobotAssertionManager assertionManager) {
        super(drive, pose, pf, headingModuleFactory, assertionManager);
        this.pose = pose;
        this.drive = drive;
        this.coprocessorComms = coprocessorComms;
        this.lastSetCoordinates = new ArrayList<XTableValues.Coordinate>();
    }

    @Override
    public void initialize() {
        super.initialize();
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

        // keep as reminder: if we change the command to poll continously we will need to reset everytime we update keypoints
        // likely not going to happen, this command will stay as is and a new command will take movement vectors instead 
        // reset();
    }

    protected boolean setTargetPoseForVision(Pose2d targetPose) {
        XTablesClientManager xTablesClientManager = this.coprocessorComms.getXTablesManager();
        XTablesClient xclient = xTablesClientManager.getOrNull();
        if (xclient == null) {
            log.warn("XTablesClientManager returned null from getXTablesClient. Client possibly waiting to find server...");
            cancel();
            return false;
        }
        // both potentialy null. Will not do anything if coordinates is null, but can proceed if heading is null
        var successful = xclient.putPose2d(this.coprocessorComms.getXtablesTargetPose(), targetPose);
        if (!successful) {
            log.warn("Failed to send to xtables target pose");
            cancel();
            return false;
        }
        this.aKitLog.record("Set target pose", targetPose);

        return true;
    }

    public static <T, U> List<Map.Entry<T, U>> zipList(List<T> list1, List<U> list2) {
        return IntStream.range(0, Math.min(list1.size(), list2.size()))
            .mapToObj(i -> new AbstractMap.SimpleEntry<T, U>(list1.get(i), list2.get(i)))
            .collect(Collectors.toList());
    }

    public static <T> boolean areListsEqual(List<T> list1, List<T> list2) {
        if (list1.size() != list2.size()) {
            return false;
        }

        var zippedList = zipList(list1, list2);

        for (var zipped : zippedList) {
            if (zipped.getKey() != zipped.getValue()) {
                return false;
            }
        }

        return true;
    }

    //allows for driving not in a straight line
    public boolean retrieveWaypointsFromVision() {
        XTablesClientManager xTablesClientManager = this.coprocessorComms.getXTablesManager();
        XTablesClient xclient = xTablesClientManager.getOrNull();
        if (xclient == null) {
            log.warn("XTablesClientManager returned null from getXTablesClient. Client possibly waiting to find server...");
            cancel();
            return false;
        }
        // both potentialy null. Will not do anything if coordinates is null, but can proceed if heading is null
        List<XTableValues.Coordinate> coordinates = xclient.getCoordinates(this.coprocessorComms.getXtablesCoordinateLocation());
        if (coordinates == null) {
            // fail
            log.warn("No coordinates found in vision.");
            cancel();
            return false;
        }
        if (areListsEqual(coordinates, this.lastSetCoordinates)) {
            log.info("Skipping setting new coordinates since existing matches pulled coordinates.");
            return true;
        }
        this.lastSetCoordinates = coordinates;
        log.info("Ingested waypoints, preparing to drive.");
        Translation2d[] waypoints = new Translation2d[coordinates.size()];
        for (int i = 0; i < coordinates.size(); i++) {
            XTableValues.Coordinate coordinate = coordinates.get(i);
            waypoints[i] = new Translation2d(coordinate.getX(), coordinate.getY());
        }
        this.aKitLog.record("Used waypoints", waypoints);

        Double heading = xclient.getDouble(this.coprocessorComms.getXtablesHeadingLocation());
        Rotation2d rotation = null;
        if (heading != null) {
            rotation = Rotation2d.fromRadians(heading);
        }

        this.aKitLog.record("Used heading", rotation);


        this.prepareToDriveWithWaypoints(waypoints, rotation);
        return true;
    }

    @Override
    public void execute() {
        if (retrieveWaypointsFromVision()) {
            super.execute();
        }
    }

    @Override
    public boolean isFinished() {
        return super.isFinished();
    }

}
