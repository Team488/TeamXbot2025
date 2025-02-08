package competition.subsystems.drive;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;

import org.junit.Test;

import competition.BaseCompetitionTest;

public class DriveSubsystemTest extends BaseCompetitionTest {
    @Test
    public void testDriveSubsystem() {
        DriveSubsystem driveSubsystem = (DriveSubsystem)getInjectorComponent().driveSubsystem();
        assertNotNull(driveSubsystem);

        driveSubsystem.refreshDataFrame();
        driveSubsystem.periodic();

        var moduleStates = driveSubsystem.getSwerveModuleStates();
        assertEquals(4, moduleStates.length);

        for (var moduleState : moduleStates) {
            assertEquals(0, moduleState.speedMetersPerSecond, 0.001);
            assertEquals(0, moduleState.angle.getDegrees(), 0.001);
        }

        assertNotNull(driveSubsystem.getFrontLeftSwerveModuleSubsystem().getDriveSubsystem().getMotorController());
    }
}
