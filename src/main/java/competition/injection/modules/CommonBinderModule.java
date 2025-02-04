package competition.injection.modules;

import competition.electrical_contract.ElectricalContract;
import competition.subsystems.coral_scorer.CoralScorerSubsystem;
import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.oracle.contracts.CoralCollectionInfoSource;
import competition.subsystems.pose.PoseSubsystem;
import competition.subsystems.vision.AprilTagVisionSubsystemExtended;
import dagger.Binds;
import dagger.Module;
import xbot.common.injection.electrical_contract.XCameraElectricalContract;
import xbot.common.injection.electrical_contract.XSwerveDriveElectricalContract;
import xbot.common.subsystems.drive.BaseDriveSubsystem;
import xbot.common.subsystems.drive.BaseSwerveDriveSubsystem;
import xbot.common.subsystems.pose.BasePoseSubsystem;
import xbot.common.subsystems.vision.AprilTagVisionSubsystem;

import javax.inject.Singleton;

/**
 * This module is used to bind the common implementations that apply to all robot variants.
 * @implNote Bindings and providers cannot be defined in the same class.
 * @see CommonProviderModule Equivalent module for providers instead of bindings.
 */
@Module
public abstract class CommonBinderModule {
    @Binds
    @Singleton
    public abstract XSwerveDriveElectricalContract getSwerveContract(ElectricalContract impl);

    @Binds
    @Singleton
    public abstract XCameraElectricalContract getCameraContract(ElectricalContract impl);

    @Binds
    @Singleton
    public abstract BaseSwerveDriveSubsystem getSwerveDriveSubsystem(DriveSubsystem impl);

    @Binds
    @Singleton
    public abstract BaseDriveSubsystem getDriveSubsystem(BaseSwerveDriveSubsystem impl);

    @Binds
    @Singleton
    public abstract BasePoseSubsystem getPoseSubsystem(PoseSubsystem impl);

    @Binds
    @Singleton
    public abstract AprilTagVisionSubsystem getVisionSubsystem(AprilTagVisionSubsystemExtended impl);

    @Binds
    @Singleton
    public abstract CoralCollectionInfoSource getCoralCollectionInfoSource(CoralScorerSubsystem impl);
}
