package competition.subsystems.drive.commands;

import competition.operator_interface.OperatorInterface;
import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.pose.PoseSubsystem;
import competition.subsystems.vision.AprilTagVisionSubsystemExtended;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import xbot.common.command.BaseCommand;
import xbot.common.math.XYPair;
import xbot.common.subsystems.drive.control_logic.HeadingModule;

import javax.inject.Inject;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

public class AlignToNearestReefFaceForAlgaeCommand extends BaseCommand {

    final HeadingModule headingModule;

    final AprilTagVisionSubsystemExtended vision;
    final DriveSubsystem drive;
    final PoseSubsystem pose;
    final OperatorInterface oi;

    Translation2d idealFinalPosition;
    Angle idealFinalHeading;

    @Inject
    public AlignToNearestReefFaceForAlgaeCommand(HeadingModule.HeadingModuleFactory headingModuleFactory,
                                                 DriveSubsystem drive, PoseSubsystem pose,
                                                 AprilTagVisionSubsystemExtended vision, OperatorInterface oi) {
        this.headingModule = headingModuleFactory.create(drive.getRotateToHeadingPid());
        this.vision = vision;
        this.drive = drive;
        this.pose = pose;
        this.oi = oi;
        this.addRequirements(drive);
    }

    @Override
    public void initialize() {
        log.info("Initializing");

        Pose2d closestPose = pose.getClosestReefFacePose();
        idealFinalPosition = closestPose.getTranslation();
        idealFinalHeading = closestPose.getRotation().getMeasure().plus(Degrees.of(180));
    }

    @Override
    public void execute() {
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
        XYPair fieldVectorXYPair = new XYPair(fieldVectorTranslation2d.getX(), fieldVectorTranslation2d.getY());
        double railsSimilarityToDriver = fieldVectorXYPair.dotProduct(
                new XYPair(
                        Math.cos(idealFinalHeading.in(Radians) + Math.PI),
                        Math.sin(idealFinalHeading.in(Radians) + Math.PI)
                )
        );

        double rotateIntent = headingModule.calculateHeadingPower(idealFinalHeading.in(Degrees));
        XYPair onRailsVector = XYPair.fromPolar(idealFinalHeading.in(Degrees), railsSimilarityToDriver);
        var combinedVector = onRailsVector.clone().add(centeringVector);

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

        double power = -drive.getDriveToAlgaePidManager().calculate(0, goalMagnitude);

        return normalizedGoalVector.times(power);
    }
}