package competition.subsystems.drive.commands;

import competition.operator_interface.OperatorInterface;
import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.oracle.ReefCoordinateGenerator;
import competition.subsystems.pose.Landmarks;
import competition.subsystems.pose.PoseSubsystem;
import competition.subsystems.vision.AprilTagVisionSubsystemExtended;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import xbot.common.command.BaseCommand;
import xbot.common.math.MathUtils;
import xbot.common.math.PIDDefaults;
import xbot.common.math.PIDManager;
import xbot.common.math.XYPair;
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

    final DoubleProperty interstitialThresholdToRailDrive;
    final DoubleProperty interstitialDistance;
    final DoubleProperty interstitialApproachSpeedFactor;

    public enum AlignmentState {
        ToInterstitial,
        RailDrive,
    }

    public AlignmentState currentAlignmentState;
    public Pose2d interstitialPoint;

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

        interstitialThresholdToRailDrive = pf.createPersistentProperty("InterstitialThresholdToRailDrive-m", 0.2);
        interstitialDistance = pf.createPersistentProperty("InterstitialDistance-m", 1.75);
        interstitialApproachSpeedFactor = pf.createPersistentProperty("InterstitialApproachSpeedFactor", 0.5);
    }

    @Override
    public void initialize() {
        log.info("Initializing");

        // Firstly, figure out if we are on red/blue side of the field, and we'll center accordingly
        Pose2d currentPose = pose.getCurrentPose2d();

        DriverStation.Alliance alliance = DriverStation.Alliance.Blue;
        if (currentPose.getTranslation().getX() > PoseSubsystem.fieldXMidpointInMeters.in(Meters)) {
            alliance = DriverStation.Alliance.Red;
        }

        Pose2d closestPose = pose.getClosestReefFacePoseByAlliance(alliance);
        idealFinalPosition = closestPose.getTranslation();
        idealFinalHeading = closestPose.getRotation().getMeasure().plus(Degrees.of(180)); // Aligning backwards

        double distanceToClosestReefFace = currentPose.getTranslation().getDistance(idealFinalPosition);

        // Determine our starting state, do we need to drive to an interstitial?
        if (distanceToClosestReefFace < interstitialDistance.get()) {
            currentAlignmentState = AlignmentState.RailDrive;
        } else {
            currentAlignmentState = AlignmentState.ToInterstitial;
            interstitialPoint = reefCoordinateGenerator.getPoseRelativeToReefFace(
                    alliance,
                    Landmarks.getReefFaceFromTagId(vision.getClosestTagIdFromTranslation(idealFinalPosition)),
                    Meters.of(interstitialDistance.get()),
                    Meters.zero()
            );
        }
    }

    @Override
    public void execute() {
        switch (currentAlignmentState) {
            case ToInterstitial -> {
                driveToInterstitial();
                var currentTranslation = pose.getCurrentPose2d().getTranslation();
                double distanceFromInterstitial = currentTranslation.getDistance(interstitialPoint.getTranslation());
                if (distanceFromInterstitial < interstitialThresholdToRailDrive.get()) {
                    currentAlignmentState = AlignmentState.RailDrive;
                }
            }
            case RailDrive -> railDrive();
            default -> {}
        }
    }

    public void driveToInterstitial() {
        Pose2d currentPose = pose.getCurrentPose2d();

        // Same interstitial driving logic as AlignCameraToAprilTagCalculator
        var vectorTowardsInterstitial = interstitialPoint.getTranslation().minus(currentPose.getTranslation());
        var normalizedVector = vectorTowardsInterstitial.div(vectorTowardsInterstitial.getNorm());
        var driveIntent = new XYPair(
                normalizedVector.getX(),
                normalizedVector.getY()).scale(interstitialApproachSpeedFactor.get()
        );
        var rotationIntent = headingModule.calculateHeadingPower(interstitialPoint.getRotation());

        drive.fieldOrientedDrive(
                driveIntent,
                rotationIntent,
                pose.getCurrentHeading().getDegrees(),
                new XYPair()
        );
    }

    private void railDrive() {
        // Calculate the vector to center our robot on "rails"
        var currentTranslation = pose.getCurrentPose2d().getTranslation();
        var currentHeading = pose.getCurrentHeading().getRadians();
        double deltaX = idealFinalPosition.getX() - currentTranslation.getX();
        double deltaY = idealFinalPosition.getY() - currentTranslation.getY();
        double robotRelativeTagLocationY = deltaX * Math.sin(-currentHeading) + deltaY * Math.cos(-currentHeading);

        // Everything below here can and will be extracted because it is repeated code with
        // DriveWithSnapToReefTagCommand
        var centeringTranslation2d = getPowerForRelativePositionChange(new Translation2d(0, robotRelativeTagLocationY));
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

        drive.fieldOrientedDrive(
                combinedVector,
                rotateIntent,
                pose.getCurrentHeading().getDegrees(),
                new XYPair()
        );
    }

    public Translation2d getPowerForRelativePositionChange(Translation2d goalPosition) {
        double goalMagnitude = goalPosition.getNorm();

        if (Math.abs(goalMagnitude) <  0.0001) {
            goalMagnitude = 0.0001;
        }
        Translation2d normalizedGoalVector = goalPosition.div(goalMagnitude);

        double power = -driveToAlgaePidManager.calculate(0, goalMagnitude);

        return normalizedGoalVector.times(power);
    }
}