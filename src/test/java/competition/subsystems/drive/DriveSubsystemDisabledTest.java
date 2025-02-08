package competition.subsystems.drive;

import competition.BaseCompetitionTest;
import competition.electrical_contract.UnitTestContract2025;
import competition.injection.components.CompetitionTestComponent;
import competition.injection.components.DaggerCompetitionTestComponent;
import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;

public class DriveSubsystemDisabledTest extends BaseCompetitionTest {
    @Override
    protected CompetitionTestComponent createDaggerComponent() {
        return (CompetitionTestComponent) DaggerCompetitionTestComponent
                .builder()
                .electricalContract(new UnitTestContract2025() {
                    @Override
                    public boolean isDriveReady() {
                        return false;
                    }

                    @Override
                    public boolean areCanCodersReady() {
                        return false;
                    }
                })
                .build();
    }

    @Test
    public void testDriveSubsystemWithDriveDisabled() {
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

        assertNull(driveSubsystem.getFrontLeftSwerveModuleSubsystem().getDriveSubsystem().getMotorController());
    }
}
