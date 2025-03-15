package competition.simulation;

import javax.inject.Inject;

import competition.subsystems.deadwheel.DeadWheelSubsystem;
import competition.subsystems.pose.PoseSubsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Rotations;

import xbot.common.advantage.AKitLogger;
import xbot.common.controls.sensors.mock_adapters.MockEncoder;
import xbot.common.properties.PropertyFactory;

public class DeadwheelsSimulator {
    final DeadWheelSubsystem deadWheelSubsystem;
    final PoseSubsystem pose;
    Pose2d lastReadPose;

    @Inject
    public DeadwheelsSimulator(DeadWheelSubsystem deadWheelSubsystem, PoseSubsystem pose) {
        super();
        this.pose = pose;
        this.deadWheelSubsystem = deadWheelSubsystem;
    }

    private MockEncoder getLeftEncoder() {
        return (MockEncoder) this.deadWheelSubsystem.leftEncoder;
    }

    private MockEncoder getRightEncoder() {
        return (MockEncoder) this.deadWheelSubsystem.rightEncoder;
    }

    private MockEncoder getFrontEncoder() {
        return (MockEncoder) this.deadWheelSubsystem.frontEncoder;
    }

    private MockEncoder getRearEncoder() {
        return (MockEncoder) this.deadWheelSubsystem.rearEncoder;
    }

    public void resetPose(Pose2d currentPose) {
        this.lastReadPose = this.pose.getCurrentPose2d();

        this.getLeftEncoder().setDistance(0);
        this.getRightEncoder().setDistance(0);
        this.getFrontEncoder().setDistance(0);
        this.getRearEncoder().setDistance(0);
    }
        

    public void update(Pose2d currentPose) {
        var deltaXGlobal = currentPose.getX() - this.lastReadPose.getX();
        var deltaYGlobal = currentPose.getY() - this.lastReadPose.getY();
        var deltaTheta = currentPose.getRotation().getRadians() - this.lastReadPose.getRotation().getRadians();

        var deltaTranslation = new Translation2d(deltaXGlobal, deltaYGlobal);
        var deltaRobotRelative = deltaTranslation.rotateBy(this.lastReadPose.getRotation().times(-1));

        var deltaFinalRelative = new Translation2d(deltaRobotRelative.getX(), deltaRobotRelative.getY()).rotateBy(Rotation2d.fromRadians(-deltaTheta));
        var deltaXFinal = deltaFinalRelative.getX();
        var pulsesInX = (deltaXFinal / 0.032) * 500;
        var deltaYFinal = deltaFinalRelative.getY();
        var pulsesInY = (deltaYFinal / 0.032) * 500;

        this.getLeftEncoder().addDistance(pulsesInY);
        this.getRightEncoder().addDistance(pulsesInY);
        this.getFrontEncoder().addDistance(pulsesInX);
        this.getRearEncoder().addDistance(pulsesInX);

        this.lastReadPose = currentPose;
    }
}
