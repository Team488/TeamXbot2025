package competition.subsystems.drive;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotEquals;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.junit.Ignore;
import org.junit.Test;

import competition.BaseCompetitionTest;

public class DriveSubsystemTest extends BaseCompetitionTest {
    @Test
    public void testDriveSubsystem() {
        DriveSubsystem driveSubsystem = (DriveSubsystem)getInjectorComponent().driveSubsystem();
        assertNotEquals(driveSubsystem, null);
    }

    @Test
    //@Ignore("Tests to help understand the behavior of WPI classes")
    public void testTransformation() {

        Transform2d robotRelativeTranslation = new Transform2d(3,5,new Rotation2d());
        Pose2d robotPositionOnField = new Pose2d(10,10, Rotation2d.fromDegrees(90));

        var fieldRelativeTranslation = robotPositionOnField.transformBy(robotRelativeTranslation);
    }
}
