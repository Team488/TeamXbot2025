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

import javax.inject.Inject;
import java.util.Arrays;
import java.util.List;
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
    int cameraToUse;
    double idealFinalHeadingDegrees;
    int loopsWithTargetCounter = 0;
    boolean isDriverRelative = false;
    boolean hasCameraFlippedDriverRelative = false;

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

        this.addRequirements(drive);
    }

    public void setDriverRelative(boolean isEnabled) {
        this.isDriverRelative = isEnabled;
    }

    public void setCameraToUse(int cameraToUse) {
        this.cameraToUse = cameraToUse;
    }

    @Override
    public void initialize() {
        log.info("Initializing");
        swerveLogic.initialize();

        // For now, we'll set our aprilTag and camera upon initialization
        if (isDriverRelative) {
            setDriverRelativeCameraToUse();
        }
        this.chosenTagID = vision.getTargetAprilTagID(pose.getClosestReefFacePose());

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
        boolean targetInSight = vision.doesCameraBestObservationHaveAprilTagId(cameraToUse, chosenTagID);

        aKitLog.record("targetInSight", targetInSight);

        if (targetInSight) {
            loopsWithTargetCounter++;
            // TODO: trigger vibration as well
            oi.driverGamepad.getRumbleManager().rumbleGamepad(0.5, 1);
        } else {
            loopsWithTargetCounter--;
            // TODO: stop any vibration.
            oi.driverGamepad.getRumbleManager().stopGamepadRumble();
        }

        if (loopsWithTargetCounter <= 0) {
            // We are in regular driving state.
            loopsWithTargetCounter = 0;
            driveAdvice = swerveLogic.getDriveAdvice();
            state = SnapState.Regular;
        }

        if (loopsWithTargetCounter > 0) {
            // We are in either LockedOn or Lost state.
            var centeringVector = new XYPair();
            if (targetInSight) {
                state = SnapState.LockedOn;
                // Use PID to center the tag in the camera.
                // First, where is this tag?
                double robotRelativeTagLocationY = vision.getRobotRelativeLocationOfBestDetectedAprilTag(cameraToUse).getY();
                var centeringTranslation2d = drive.getPowerForRelativePositionChange(new Translation2d(0, robotRelativeTagLocationY));
                centeringVector = new XYPair(centeringTranslation2d.getX(), centeringTranslation2d.getY());
                // All of that was robot-relative. Rotate into field-relative.
                centeringVector = centeringVector.rotate(pose.getCurrentHeading().getDegrees());
            } else {
                state = SnapState.Lost;
            }

            // Whether we have it or have lost it, we still want to drive forward/backward relative to the robot based
            // on driver joystick input.
            var fieldVectorTranslation2d = oi.driverGamepad.getLeftFieldOrientedVector();
            aKitLog.record("Driver Vector Angle", fieldVectorTranslation2d.getAngle().getDegrees());
            aKitLog.record("Ideal Angle", idealFinalHeadingDegrees);

            XYPair fieldVectorXYPair = new XYPair(fieldVectorTranslation2d.getX(), fieldVectorTranslation2d.getY());
            double railsSimilarityToDriver = fieldVectorXYPair.dotProduct(
                    new XYPair(
                            Math.cos(Math.toRadians(idealFinalHeadingDegrees) + Math.PI),
                            Math.sin(Math.toRadians(idealFinalHeadingDegrees) + Math.PI)
                    )
            );

            aKitLog.record("railsSimilarityToDriver", railsSimilarityToDriver);

            double rotateIntent = headingModule.calculateHeadingPower(idealFinalHeadingDegrees);
            XYPair onRailsVector = XYPair.fromPolar(idealFinalHeadingDegrees, railsSimilarityToDriver);

            aKitLog.record("onRailsVector", onRailsVector);
            aKitLog.record("centeringVector", centeringVector);


            var combinedVector = onRailsVector.clone().add(centeringVector);

            driveAdvice = new ManualSwerveDriveAdvice(
                    combinedVector,
                    rotateIntent,
                    pose.getCurrentHeading().getDegrees(),
                    new XYPair()
            );
        }

        aKitLog.record("LoopsWithTargetCounter", loopsWithTargetCounter);
        aKitLog.record("state", state);

        drive.fieldOrientedDrive(
                driveAdvice.translation(),
                driveAdvice.rotationIntent(),
                driveAdvice.currentHeading(),
                driveAdvice.centerOfRotationInches()
        );
    }

    // Stolen from AlignToReefWithAprilTagCommand for convenience sakes
    private void setDriverRelativeCameraToUse() {
        List<Integer> farReefFacePoseIDList = Arrays.asList(20, 21, 22, 9, 10 , 11);
        List<Integer> closeReefFacePoseIDList = Arrays.asList(19, 18, 17, 6, 7, 8);

        // if our target april tag is a far april tag and cameras haven't been flipped,
        // flip and use the other front camera to align with tag
        if (farReefFacePoseIDList.contains(vision.getTargetAprilTagID(pose.getClosestReefFacePose()))
                && !hasCameraFlippedDriverRelative) {
            cameraToUse = (cameraToUse + 1) % 2;
            hasCameraFlippedDriverRelative = true;
        }
        // if our target april tag is a close april tag and cameras have been flipped,
        // flip and use the other front camera to align with tag
        else if (closeReefFacePoseIDList.contains(vision.getTargetAprilTagID(pose.getClosestReefFacePose()))
                && hasCameraFlippedDriverRelative) {
            cameraToUse = (cameraToUse + 1) % 2;
            hasCameraFlippedDriverRelative = false;
        }
    }
}
