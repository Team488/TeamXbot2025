package competition.subsystems.drive.commands;

import competition.operator_interface.OperatorInterface;
import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.drive.logic.ManualSwerveDriveAdvice;
import competition.subsystems.drive.logic.ManualSwerveDriveLogic;
import competition.subsystems.pose.DriverRelativeCameraValues;
import competition.subsystems.pose.PoseSubsystem;
import competition.subsystems.vision.AprilTagVisionSubsystemExtended;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
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
    Angle idealFinalHeading;
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

        // For now, we'll set our aprilTag target upon initialization
        // This may need to be changed later if target needs to update in real time.
        if (isDriverRelative) {
            setDriverRelativeCameraToUse();
        }
        this.chosenTagID = vision.getTargetAprilTagID(pose.getClosestReefFacePose());

        Optional<Pose3d> aprilTagPose = vision.getAprilTagFieldOrientedPose(chosenTagID);
        Angle aprilTagZRotation = Radians.of(aprilTagPose.map((p) -> p.getRotation().getZ()).orElse(0.0));
        aKitLog.record("TargetAprilTag", chosenTagID);
        aKitLog.record("DoesAprilTagExist", aprilTagPose.isPresent());

        idealFinalHeading = aprilTagZRotation.plus(Radians.of(Math.PI));

        state = searchForTarget() ? SnapState.LockedOn : SnapState.Regular;
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
        boolean targetInSight = searchForTarget();

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
            aKitLog.record("DriverVectorAngle-deg", fieldVectorTranslation2d.getAngle().getDegrees());
            aKitLog.record("IdealAngle-deg", idealFinalHeading.in(Degrees));

            XYPair fieldVectorXYPair = new XYPair(fieldVectorTranslation2d.getX(), fieldVectorTranslation2d.getY());
            double railsSimilarityToDriver = fieldVectorXYPair.dotProduct(
                    new XYPair(
                            Math.cos(idealFinalHeading.in(Radians) + Math.PI),
                            Math.sin(idealFinalHeading.in(Radians) + Math.PI)
                    )
            );

            aKitLog.record("railsSimilarityToDriver", railsSimilarityToDriver);

            double rotateIntent = headingModule.calculateHeadingPower(idealFinalHeading.in(Degrees));
            XYPair onRailsVector = XYPair.fromPolar(idealFinalHeading.in(Degrees), railsSimilarityToDriver);

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

    /**
     * @return target found
     */
    private boolean searchForTarget() {
        boolean targetInSight = vision.doesCameraBestObservationHaveAprilTagId(cameraToUse, chosenTagID);

        aKitLog.record("targetInSight", targetInSight);

        if (targetInSight) {
            loopsWithTargetCounter++;
            oi.driverGamepad.getRumbleManager().rumbleGamepad(0.5, 0.05);
        } else {
            loopsWithTargetCounter--;
            oi.driverGamepad.getRumbleManager().stopGamepadRumble();
        }

        return targetInSight;
    }

    private void setDriverRelativeCameraToUse() {
        DriverRelativeCameraValues values = pose.getDriverRelativeCameraToUse(hasCameraFlippedDriverRelative, cameraToUse);
        hasCameraFlippedDriverRelative = values.hasCameraFlippedDriverRelative();
        cameraToUse = values.cameraToUse();
    }
}
