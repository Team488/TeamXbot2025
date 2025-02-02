package competition.subsystems.oracle.commands;

import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.oracle.OracleSubsystem;
import competition.subsystems.pose.PoseSubsystem;
import xbot.common.logging.RobotAssertionManager;
import xbot.common.properties.PropertyFactory;
import xbot.common.subsystems.drive.SwervePointKinematics;
import xbot.common.subsystems.drive.SwerveSimpleTrajectoryCommand;
import xbot.common.subsystems.drive.SwerveSimpleTrajectoryMode;
import xbot.common.subsystems.drive.control_logic.HeadingModule;

import javax.inject.Inject;

public class DriveUsingOracleToPickup extends SwerveSimpleTrajectoryCommand {

    OracleSubsystem oracle;

    @Inject
    public DriveUsingOracleToPickup(OracleSubsystem oracle, DriveSubsystem drive, PoseSubsystem pose, PropertyFactory pf,
                                    HeadingModule.HeadingModuleFactory headingModuleFactory,
                                    RobotAssertionManager robotAssertionManager) {
        super(drive, pose, pf, headingModuleFactory, robotAssertionManager);
        this.drive = drive;
        this.pose = pose;
        this.oracle = oracle;
    }

    @Override
    public void initialize() {
        log.info("Initializing");

        SwervePointKinematics sampleKinematics = new SwervePointKinematics(2, 0, 0, 4.5);

        logic.setKeyPoints(oracle.getRecommendedScoringTrajectory());
        this.logic.setVelocityMode(SwerveSimpleTrajectoryMode.GlobalKinematicsValue);
        this.logic.setGlobalKinematicValues(sampleKinematics);

        super.initialize();
    }

    @Override
    public void execute() {
        super.execute();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
}
