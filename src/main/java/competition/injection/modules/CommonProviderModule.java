package competition.injection.modules;

import dagger.Module;
import dagger.Provides;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import xbot.common.injection.swerve.FrontLeftDrive;
import xbot.common.injection.swerve.FrontRightDrive;
import xbot.common.injection.swerve.RearLeftDrive;
import xbot.common.injection.swerve.RearRightDrive;
import xbot.common.injection.swerve.SwerveComponent;
import xbot.common.injection.swerve.SwerveInstance;

import javax.inject.Singleton;

/**
 * This module defines providers that apply to all robot configurations.
 * @implNote Bindings and providers cannot be defined in the same class.
 * @see CommonBinderModule Equivalent module for bindings instead of providers.
 */
@Module(subcomponents = { SwerveComponent.class })
public class CommonProviderModule {
    @Provides
    @Singleton
    public @FrontLeftDrive SwerveComponent frontLeftSwerveComponent(SwerveComponent.Builder builder) {
        return builder
                .swerveInstance(new SwerveInstance("FrontLeftDrive"))
                .build();
    }

    @Provides
    @Singleton
    public @FrontRightDrive SwerveComponent frontRightSwerveComponent(SwerveComponent.Builder builder) {
        return builder
                .swerveInstance(new SwerveInstance("FrontRightDrive"))
                .build();
    }

    @Provides
    @Singleton
    public @RearLeftDrive SwerveComponent rearLeftSwerveComponent(SwerveComponent.Builder builder) {
        return builder
                .swerveInstance(new SwerveInstance("RearLeftDrive"))
                .build();
    }

    @Provides
    @Singleton
    public @RearRightDrive SwerveComponent rearRightSwerveComponent(SwerveComponent.Builder builder) {
        return builder
                .swerveInstance(new SwerveInstance("RearRightDrive"))
                .build();
    }

    @Provides
    @Singleton
    public AprilTagFieldLayout fieldLayout() {
        return AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    }
}