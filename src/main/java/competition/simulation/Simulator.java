package competition.simulation;

import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.pose.PoseSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import xbot.common.math.MovingAverageForDouble;
import xbot.common.math.MovingAverageForTranslation2d;

import javax.inject.Inject;

public class Simulator {

    MovingAverageForTranslation2d translationAverageCalculator =
            new MovingAverageForTranslation2d(15);

    MovingAverageForDouble rotationAverageCalculator =
            new MovingAverageForDouble(15);

    PoseSubsystem pose;
    DriveSubsystem drive;
    final double robotTopSpeedInMetersPerSecond = 3.0;
    final double robotLoopPeriod = 0.02;
    final double robotTopAngularSpeedInDegreesPerSecond = 360;
    final double poseAdjustmentFactorForDriveSimulation = robotTopSpeedInMetersPerSecond * robotLoopPeriod;
    final double headingAdjustmentFactorForDriveSimulation = robotTopAngularSpeedInDegreesPerSecond * robotLoopPeriod;

    @Inject
    public Simulator(PoseSubsystem pose, DriveSubsystem drive) {
        this.pose = pose;
        this.drive = drive;
    }

    public void update() {
        simulateDrive();
    }

    private void simulateDrive() {
        var currentPose = pose.getCurrentPose2d();

        // Extremely simple physics simulation. We want to give the robot some very basic translational and rotational
        // inertia. We can take the moving average of the last second or so of robot commands and apply that to the
        // robot's pose. This is a very simple way to simulate the robot's movement without having to do any real physics.

        translationAverageCalculator.add(drive.lastRawCommandedDirection);
        var currentAverage = translationAverageCalculator.getAverage();

        rotationAverageCalculator.add(drive.lastRawCommandedRotation);
        var currentRotationAverage = rotationAverageCalculator.getAverage();

        var updatedPose = new Pose2d(
                new Translation2d(
                        currentPose.getTranslation().getX() + currentAverage.getX() * poseAdjustmentFactorForDriveSimulation,
                        currentPose.getTranslation().getY() + currentAverage.getY() * poseAdjustmentFactorForDriveSimulation),
                currentPose.getRotation().plus(Rotation2d.fromDegrees(currentRotationAverage * headingAdjustmentFactorForDriveSimulation)));
        pose.setCurrentPoseInMeters(updatedPose);
    }
}
