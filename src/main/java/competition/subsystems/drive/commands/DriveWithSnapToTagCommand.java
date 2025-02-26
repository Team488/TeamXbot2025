package competition.subsystems.drive.commands;

import competition.operator_interface.OperatorInterface;
import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.drive.logic.ManualSwerveDriveAdvice;
import competition.subsystems.drive.logic.ManualSwerveDriveLogic;
import competition.subsystems.pose.PoseSubsystem;
import competition.subsystems.vision.AprilTagVisionSubsystemExtended;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import xbot.common.command.BaseCommand;
import xbot.common.math.XYPair;
import xbot.common.subsystems.drive.control_logic.HeadingModule;
import xbot.common.subsystems.vision.AprilTagVisionIO;

import javax.inject.Inject;
import java.util.Optional;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

public class DriveWithSnapToTagCommand extends BaseCommand {

    final DriveSubsystem drive;
    final AprilTagVisionSubsystemExtended vision;
    final PoseSubsystem pose;
    final OperatorInterface oi;
    final ManualSwerveDriveLogic swerveLogic;
    final HeadingModule headingModule;
    int chosenTagID;
    int cameraId;
    double idealFinalHeadingDegrees;
    int loopsWithTargetCounter = 0;

    enum SnapState {
        Regular,
        LockedOn,
        Lost
    }

    SnapState state = SnapState.Regular;

    @Inject
    public DriveWithSnapToTagCommand(DriveSubsystem drive, AprilTagVisionSubsystemExtended vision,
                                     ManualSwerveDriveLogic.ManualSwerveDriveLogicFactory factory,
                                     OperatorInterface oi, HeadingModule.HeadingModuleFactory headingModuleFactory,
                                     PoseSubsystem pose) {
        this.drive = drive;
        this.vision = vision;
        this.oi = oi;
        this.pose = pose;
        this.swerveLogic = factory.create();
        this.headingModule = headingModuleFactory.create(drive.getRotateToHeadingPid());
    }

    public void setChosenTagID(int chosenTagID) {
        this.chosenTagID = chosenTagID;
    }

    public void setCameraId(int cameraId) {
        this.cameraId = cameraId;
    }

    @Override
    public void initialize() {
        swerveLogic.initialize();
        vision.setCameraSearchMode(cameraId, AprilTagVisionIO.SearchMode.PRIORITIZE_SPECIFIC_TAG);
        vision.setCameraSpecificTagIdToSearchFor(cameraId, chosenTagID);

        Optional<Pose3d> aprilTagPose = vision.getAprilTagFieldOrientedPose(chosenTagID);
        var aprilTagZRotationRadians = aprilTagPose.map((p) -> p.getRotation().getZ()).orElse(0.0);
        idealFinalHeadingDegrees = Radians.of(Math.PI + aprilTagZRotationRadians).in(Degrees);

        state = SnapState.Regular;
        loopsWithTargetCounter = 0;
    }

    @Override
    public void execute() {
        // Basic idea is this - we are in one of three states
        // Regular: We haven't seen the tag. In this case, do swerve drive as usual.
        // LockedOn: We see the Tag. Use PID to center it to the camera, and re-interpret the
        //      driver translation joystick to only allow robot-relative forward/backward motion.
        // Lost: We saw the tag, but now we don't. We stay in this state for as long as we were in the
        //      LockedOn state to include some hysteresis. In this state, like in LockedOn, we re-interpret
        //      the driver translation joystick to only allow robot-relative forward/backward motion.

        var driveAdvice = new ManualSwerveDriveAdvice();
        boolean targetInSight = vision.doesCameraBestObservationHaveAprilTagId(cameraId, chosenTagID);

        if (targetInSight) {
            loopsWithTargetCounter++;
            // TODO: trigger vibration as well
        } else {
            loopsWithTargetCounter--;
            // TODO: stop any vibration.
        }

        if (loopsWithTargetCounter <= 0) {
            // We are in regular driving state.
            loopsWithTargetCounter = 0;
            driveAdvice = swerveLogic.getDriveAdvice();
        }

        if (loopsWithTargetCounter > 0) {
            // We are in either LockedOn or Lost state.
            var centeringVector = new XYPair();
            if (targetInSight) {
                // Use PID to center the tag in the camera.
                // First, where is this tag?
                double robotRelativeTagLocationY = vision.getRobotRelativeLocationOfBestDetectedAprilTag(cameraId).getY();
                var centeringTranslation2d = drive.getPowerForRelativePositionChange(new Translation2d(0, robotRelativeTagLocationY));
                centeringVector = new XYPair(centeringTranslation2d.getX(), centeringTranslation2d.getY());
                // All of that was robot-relative. Rotate into field-relative.
                centeringVector = centeringVector.rotate(pose.getCurrentHeading().getDegrees());
            }

            // Whether we have it or have lost it, we still want to drive forward/backward relative to the robot based
            // on driver joystick input.
            var fieldVectorTranslation2d = oi.driverGamepad.getLeftFieldOrientedVector();
            XYPair fieldVectorXYPair = new XYPair(fieldVectorTranslation2d.getX(), fieldVectorTranslation2d.getY());
            double railsSimilarityToDriver = fieldVectorXYPair.dotProduct(new XYPair(Math.cos(idealFinalHeadingDegrees), Math.sin(idealFinalHeadingDegrees)));
            double rotateIntent = headingModule.calculateHeadingPower(idealFinalHeadingDegrees);
            XYPair onRailsVector = XYPair.fromPolar(idealFinalHeadingDegrees, railsSimilarityToDriver);

            aKitLog.record("onRailsVector", onRailsVector);
            aKitLog.record("centeringVector", centeringVector);
            aKitLog.record("LoopsWithTargetCounter", loopsWithTargetCounter);

            var combinedVector = onRailsVector.clone().add(centeringVector);

            driveAdvice = new ManualSwerveDriveAdvice(
                    combinedVector,
                    rotateIntent,
                    pose.getCurrentHeading().getDegrees(),
                    new XYPair()
            );
        }

        drive.fieldOrientedDrive(
                driveAdvice.translation(),
                driveAdvice.rotationIntent(),
                driveAdvice.currentHeading(),
                driveAdvice.centerOfRotationInches()
        );
    }

    @Override
    public void end(boolean interrupted) {
        vision.setAllCamerasToRegularSearchMode();
    }
}
