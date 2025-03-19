package competition.subsystems.drive.commands;

import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.oracle.ReefRoutingCircle;
import competition.subsystems.pose.Cameras;
import competition.subsystems.pose.Landmarks;
import competition.subsystems.pose.PoseSubsystem;
import competition.subsystems.vision.AprilTagVisionSubsystemExtended;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import xbot.common.logging.RobotAssertionManager;
import xbot.common.properties.PropertyFactory;
import xbot.common.subsystems.drive.SwervePointKinematics;
import xbot.common.subsystems.drive.SwerveSimpleTrajectoryCommand;
import xbot.common.subsystems.drive.SwerveSimpleTrajectoryMode;
import xbot.common.subsystems.drive.control_logic.HeadingModule;
import xbot.common.trajectory.XbotSwervePoint;

import javax.inject.Inject;
import java.util.List;

public class DriveToTargetBranchUntilDetectionCommand extends SwerveSimpleTrajectoryCommand {

    AprilTagVisionSubsystemExtended aprilTagVisionSubsystem;
    PoseSubsystem pose;
    Pose2d currentPose;
    Pose2d targetReefFacePose;
    ReefRoutingCircle routingCircle;
    boolean kinematics = true;
    private Cameras camera = Cameras.FRONT_LEFT_CAMERA;

    @Inject
    public DriveToTargetBranchUntilDetectionCommand(DriveSubsystem drive, PoseSubsystem pose,
                                                    PropertyFactory pf,
                                                    HeadingModule.HeadingModuleFactory headingModuleFactory,
                                                    RobotAssertionManager robotAssertionManager,
                                                    AprilTagVisionSubsystemExtended aprilTagVisionSubsystem) {
        super(drive, pose, pf, headingModuleFactory, robotAssertionManager);
        this.aprilTagVisionSubsystem = aprilTagVisionSubsystem;
        this.pose = pose;
        Translation2d center = Landmarks.BlueCenterOfReef.getTranslation();
        double radius = 2.0;
        routingCircle = new ReefRoutingCircle(center, radius);
    }

    public void setTargetReefFacePose(Pose2d targetReefFacePose) {
        this.targetReefFacePose = targetReefFacePose;
    }

    public void setAprilTagCamera(Cameras camera) {
        this.camera = camera;
    }

    @Override
    public void initialize() {
        log.info("Initializing");

        currentPose = pose.getCurrentPose2d();
        List<XbotSwervePoint> swervePoints = routingCircle.generateSwervePoints(currentPose, targetReefFacePose);
        this.logic.setKeyPoints(swervePoints);
        if (kinematics) {
            this.logic.setGlobalKinematicValues(new SwervePointKinematics(2, 0, 0, 4));
            this.logic.setVelocityMode(SwerveSimpleTrajectoryMode.GlobalKinematicsValue);
        }
        else {
            this.logic.setConstantVelocity(drive.getMaxTargetSpeedMetersPerSecond());
            this.logic.setVelocityMode(SwerveSimpleTrajectoryMode.ConstantVelocity);
        }
        super.initialize();
    }

    @Override
    public boolean isFinished() {
        return aprilTagVisionSubsystem.doesCameraBestObservationHaveAprilTagId(camera.getIndex(),
                aprilTagVisionSubsystem.getTargetAprilTagID(targetReefFacePose))
                || logic.recommendIsFinished(pose.getCurrentPose2d(), drive.getPositionalPid(), headingModule);
    }
}
