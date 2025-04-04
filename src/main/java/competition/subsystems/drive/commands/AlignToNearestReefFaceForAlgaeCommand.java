package competition.subsystems.drive.commands;

import competition.operator_interface.OperatorInterface;
import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.drive.logic.ManualSwerveDriveLogic;
import competition.subsystems.oracle.ReefCoordinateGenerator;
import competition.subsystems.pose.Landmarks;
import competition.subsystems.pose.PoseSubsystem;
import competition.subsystems.vision.AprilTagVisionSubsystemExtended;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import xbot.common.command.BaseCommand;
import xbot.common.math.MathUtils;
import xbot.common.math.PIDDefaults;
import xbot.common.math.PIDManager;
import xbot.common.math.XYPair;
import xbot.common.properties.DistanceProperty;
import xbot.common.properties.DoubleProperty;
import xbot.common.properties.PropertyFactory;
import xbot.common.subsystems.drive.control_logic.HeadingModule;

import javax.inject.Inject;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

public class AlignToNearestReefFaceForAlgaeCommand extends BaseCommand {

    final HeadingModule headingModule;

    final AprilTagVisionSubsystemExtended vision;
    final DriveSubsystem drive;
    final PoseSubsystem pose;
    final OperatorInterface oi;
    final ReefCoordinateGenerator reefCoordinateGenerator;

    Translation2d idealFinalPosition;
    Angle idealFinalHeading;

    private final PIDManager driveToAlgaePidManager;
    private final PIDManager snapToAlgaePIDManager;
    int targetTagID;

    @Inject
    public AlignToNearestReefFaceForAlgaeCommand(HeadingModule.HeadingModuleFactory headingModuleFactory,
                                                 DriveSubsystem drive, PoseSubsystem pose,
                                                 AprilTagVisionSubsystemExtended vision, OperatorInterface oi,
                                                 PIDManager.PIDManagerFactory pidFactory, PropertyFactory pf,
                                                 ReefCoordinateGenerator reefCoordinateGenerator) {
        this.vision = vision;
        this.drive = drive;
        this.pose = pose;
        this.oi = oi;
        this.reefCoordinateGenerator = reefCoordinateGenerator;
        this.addRequirements(drive);

        driveToAlgaePidManager = pidFactory.create(
                this.getPrefix() + "DriveToAlgaePositionalPID",
                new PIDDefaults(
                        0.75, // P
                        0, // I
                        4.0, // D
                        0.0, // F
                        0.6, // Max output
                        -0.6, // Min output
                        0.05, // Error threshold
                        0.005, // Derivative threshold
                        0.2) // Time threshold
        );
        driveToAlgaePidManager.setEnableErrorThreshold(true);
        driveToAlgaePidManager.setEnableTimeThreshold(true);

        snapToAlgaePIDManager = pidFactory.create(
                this.getPrefix() + "SnapToAlgaeHeadingPID",
                new PIDDefaults(
                        0.005, // P
                        0, // I
                        0, // D
                        0.0, // F
                        0.75, // Max output
                        -0.75, // Min output
                        2.0, // Error threshold
                        0.2, // Derivative threshold
                        0.2) // Time threshold
        );
        snapToAlgaePIDManager.setEnableErrorThreshold(true);
        snapToAlgaePIDManager.setEnableTimeThreshold(true);

        this.headingModule = headingModuleFactory.create(snapToAlgaePIDManager);

        pf.setPrefix(this.getPrefix());
    }

    @Override
    public void initialize() {
        log.info("Initializing");
        Pose2d currentPose = pose.getCurrentPose2d();

        // Firstly, figure out if we are on red/blue side of the field, and we'll center accordingly
        // There exist a possibility that we want to steal algae on the opposition's reef (although it goes both ways)
        DriverStation.Alliance alliance = DriverStation.Alliance.Blue;
        if (currentPose.getTranslation().getX() > PoseSubsystem.fieldXMidpointInMeters.in(Meters)) {
            alliance = DriverStation.Alliance.Red;
        }

        Pose2d closestPose = pose.getClosestReefFacePoseByAlliance(alliance);
        targetTagID = vision.getTargetAprilTagID(closestPose);

        idealFinalPosition = closestPose.getTranslation();
        idealFinalHeading = closestPose.getRotation().getMeasure().plus(Degrees.of(180)); // Aligning backwards
    }

    @Override
    public void execute() {
        railDrive();
    }

    private void railDrive() {
        // Calculate the vector to center our robot on "rails"
        boolean targetInSight = vision.doesCameraBestObservationHaveAprilTagId(2, targetTagID);
        aKitLog.record("targetInSight", targetInSight);
        var currentTranslation = pose.getCurrentPose2d().getTranslation();
        if (!targetInSight) {
            // Try and look at the thingy based off of pose
            var targetPose = vision.getAprilTagFieldOrientedPose(targetTagID);
            if (targetPose.isEmpty()) {
                log.info("Stuff happened");
                return;
            }
            double desiredHeading = currentTranslation.minus(targetPose.get().toPose2d().getTranslation())
                    .getAngle().getDegrees();
            aKitLog.record("desiredHeading", desiredHeading);

            drive.fieldOrientedDrive(
                    new XYPair(),
                    headingModule.calculateHeadingPower(desiredHeading),
                    pose.getCurrentHeading().getDegrees(),
                    new XYPair()
            );
            return;
        }

        double robotRelativeTagLocationY = vision.getRobotRelativeLocationOfBestDetectedAprilTag(2).getY() - 0.55;
        aKitLog.record("robotRelativeTagLocationY", robotRelativeTagLocationY);

        // Everything below here can and should be extracted because it is repeated code with
        // DriveWithSnapToReefTagCommand
        var centeringTranslation2d = drive.getPowerForRelativePositionChange(
                driveToAlgaePidManager,
                new Translation2d(0, robotRelativeTagLocationY)
        );
        XYPair centeringVector = new XYPair(centeringTranslation2d.getX(), centeringTranslation2d.getY());
        centeringVector = centeringVector.rotate(pose.getCurrentHeading().getDegrees());

        var fieldVectorTranslation2d = oi.driverGamepad.getLeftFieldOrientedVector();
        XYPair fieldVectorXYPair = new XYPair(
                MathUtils.deadband(fieldVectorTranslation2d.getX(), 0.05),
                MathUtils.deadband(fieldVectorTranslation2d.getY(), 0.05)
        );
        double railsSimilarityToDriver = fieldVectorXYPair.dotProduct(
                new XYPair(
                        Math.cos(idealFinalHeading.in(Radians) + Math.PI),
                        Math.sin(idealFinalHeading.in(Radians) + Math.PI)
                )
        );

        double rotateIntent = headingModule.calculateHeadingPower(idealFinalHeading.in(Degrees));
        XYPair onRailsVector = XYPair.fromPolar(idealFinalHeading.in(Degrees), railsSimilarityToDriver);
        var combinedVector = onRailsVector.add(centeringVector);
        aKitLog.record("rotateIntent", rotateIntent);
        aKitLog.record("onRailsVector", onRailsVector);
        aKitLog.record("centeringVector", centeringVector);
        aKitLog.record("combinedVector", combinedVector);

        drive.fieldOrientedDrive(
                combinedVector,
                rotateIntent,
                pose.getCurrentHeading().getDegrees(),
                new XYPair()
        );
    }
}