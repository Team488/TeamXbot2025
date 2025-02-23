package competition.subsystems.vision;

import competition.BaseCompetitionTest;

import static edu.wpi.first.units.Units.Inches;
import static org.junit.Assert.assertEquals;

import competition.subsystems.drive.logic.AlignCameraToAprilTagCalculator;
import competition.subsystems.pose.PoseSubsystem;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;
import org.junit.Test;
import xbot.common.injection.electrical_contract.CameraInfo;
import xbot.common.subsystems.vision.CameraCapabilities;

import java.util.EnumSet;

public class AlignCameraToAprilTagCalculatorTest extends BaseCompetitionTest {

    @Test
    public void testAlignmentPointOffset() {
        // Tests to make sure the generateAlignmentPointOffset method properly calculates the offset
        // between the cameras and april tags. Takes in consideration of bumper, camera location, and a manual offset

        Distance robotCenterToOuterBumperX = Inches.of(10);

        CameraInfo mockForwardCameraInfo = new CameraInfo(
                "MockCamera",
                "MockCamera",
                new Transform3d(new Translation3d(5 / PoseSubsystem.INCHES_IN_A_METER, 0, 0), new Rotation3d()),
                EnumSet.of(CameraCapabilities.APRIL_TAG)
        );

        Translation2d forwardCameraOffset = AlignCameraToAprilTagCalculator.generateAlignmentPointOffset(
                robotCenterToOuterBumperX,
                mockForwardCameraInfo,
                Inches.of(10),
                false
        );

        assertEquals(15 / PoseSubsystem.INCHES_IN_A_METER, forwardCameraOffset.getX(), 0.001);

        CameraInfo mockBackwardCameraInfo = new CameraInfo(
                "MockCamera",
                "MockCamera",
                new Transform3d(
                        new Translation3d(-5 / PoseSubsystem.INCHES_IN_A_METER, 0, 0),
                        new Rotation3d(0, 0, Math.PI)),
                EnumSet.of(CameraCapabilities.APRIL_TAG)
        );

        Translation2d backwardCameraOffset = AlignCameraToAprilTagCalculator.generateAlignmentPointOffset(
                robotCenterToOuterBumperX,
                mockBackwardCameraInfo,
                Inches.of(10),
                true
        );

        assertEquals(-15 / PoseSubsystem.INCHES_IN_A_METER, backwardCameraOffset.getX(), 0.001);
    }

}
