package competition.subsystems.drive;

import javax.inject.Inject;
import javax.inject.Singleton;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import xbot.common.injection.swerve.FrontLeftDrive;
import xbot.common.injection.swerve.FrontRightDrive;
import xbot.common.injection.swerve.RearLeftDrive;
import xbot.common.injection.swerve.RearRightDrive;
import xbot.common.injection.swerve.SwerveComponent;
import xbot.common.math.PIDManager.PIDManagerFactory;
import xbot.common.math.XYPair;
import xbot.common.properties.DoubleProperty;
import xbot.common.properties.Property;
import xbot.common.properties.PropertyFactory;
import xbot.common.subsystems.drive.BaseSwerveDriveSubsystem;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

    @Singleton
    public class DriveSubsystem extends BaseSwerveDriveSubsystem {
        private static Logger log = LogManager.getLogger(DriveSubsystem.class);

        private Translation2d lookAtPointTarget = new Translation2d(); // The target point to look at
        private Rotation2d staticHeadingTarget = new Rotation2d(); // The heading you want to constantly be at
        private boolean lookAtPointActive = false;
        private boolean staticHeadingActive = false;

        private final SysIdRoutine sysIdDrive;
        private final SysIdRoutine sysIdRotation;
        private final DoubleProperty driveToWaypointsSpeed;
        private final DoubleProperty driveToWaypointsDurationPerPoint;

        @Inject
        public DriveSubsystem(PIDManagerFactory pidFactory, PropertyFactory pf,
                              @FrontLeftDrive SwerveComponent frontLeftSwerve, @FrontRightDrive SwerveComponent frontRightSwerve,
                              @RearLeftDrive SwerveComponent rearLeftSwerve, @RearRightDrive SwerveComponent rearRightSwerve) {

            super(pidFactory, pf, frontLeftSwerve, frontRightSwerve, rearLeftSwerve, rearRightSwerve);
            log.info("Creating DriveSubsystem");

            pf.setPrefix(this.getPrefix());
            pf.setDefaultLevel(Property.PropertyLevel.Important);

            this.sysIdDrive = new SysIdRoutine(
                    new SysIdRoutine.Config(
                            Volts.of(getMaxTargetSpeedMetersPerSecond() / 5).per(Second),
                            Volts.of(getMaxTargetSpeedMetersPerSecond()),
                            Seconds.of(6),
                            (state) -> org.littletonrobotics.junction.Logger.recordOutput(this.getPrefix() + "/SysIdState-Drive", state.toString())),
                    new SysIdRoutine.Mechanism(
                            (Voltage volts) -> move(new XYPair(volts.in(Volts), 0), 0),
                            null,
                            this
                    )
            );

            this.sysIdRotation = new SysIdRoutine(
                    new SysIdRoutine.Config(
                            Volts.of(getMaxTargetTurnRate() / 5).per(Second),
                            Volts.of(getMaxTargetTurnRate()),
                            Seconds.of(6),
                            (state) -> org.littletonrobotics.junction.Logger.recordOutput(this.getPrefix() + "/SysIdState-Rotation", state.toString())),
                    new SysIdRoutine.Mechanism(
                            (Voltage volts) -> move(new XYPair(), volts.in(Volts)),
                            null,
                            this
                    )
            );

            driveToWaypointsSpeed = pf.createPersistentProperty("Speed to drive to waypoints", 2); // meters/s
            driveToWaypointsDurationPerPoint = pf.createPersistentProperty("Time to drive to waypoints", 0.1); // seconds
        }

        public Translation2d getLookAtPointTarget() {
            return lookAtPointTarget;
        }

        public Rotation2d getStaticHeadingTarget() {
            return staticHeadingTarget;
        }

        public boolean getLookAtPointActive() {
            return lookAtPointActive;
        }

        public boolean getStaticHeadingActive() {
            return staticHeadingActive;
        }

        public void setStaticHeadingTarget(Rotation2d staticHeadingTarget) {
            this.staticHeadingTarget = staticHeadingTarget;
        }

        public void setLookAtPointTarget(Translation2d lookAtPointTarget) {
            this.lookAtPointTarget = lookAtPointTarget;
        }

        public void setStaticHeadingTargetActive(boolean staticHeadingActive) {
            this.staticHeadingActive = staticHeadingActive;
        }

        public void setLookAtPointTargetActive(boolean lookAtPointActive) {
            this.lookAtPointActive = lookAtPointActive;
        }

        public InstantCommand createSetStaticHeadingTargetCommand(Supplier<Rotation2d> staticHeadingTarget) {
            return new InstantCommand(() -> {
                setStaticHeadingTarget(staticHeadingTarget.get());
                setStaticHeadingTargetActive(true);}
            );
        }

        public InstantCommand createSetLookAtPointTargetCommand(Supplier<Translation2d> lookAtPointTarget) {
            return new InstantCommand(() -> {
                setLookAtPointTarget(lookAtPointTarget.get());
                setLookAtPointTargetActive(true);}
            );
        }

        public InstantCommand createClearAllHeadingTargetsCommand() {
            return new InstantCommand(() -> {
                setStaticHeadingTargetActive(false);
                setLookAtPointTargetActive(false);
            });
        }

        /**
         * Gets a command to run the SysId drive routine in the quasistatic mode.
         * @param direction The direction to run the SysId routine.
         * @return The command to run the SysId routine.
         */
        public Command sysIdQuasistaticDrive(SysIdRoutine.Direction direction) {
            return sysIdDrive.quasistatic(direction);
        }

        /**
         * Gets a command to run the SysId drive routine in the dynamic mode.
         * @param direction The direction to run the SysId routine.
         * @return The command to run the SysId routine.
         */
        public Command sysIdDynamicDrive(SysIdRoutine.Direction direction) {
            return sysIdDrive.dynamic(direction);
        }

        /**
         * Gets a command to run the SysId rotation routine in the quasistatic mode.
         * @param direction The direction to run the SysId routine.
         * @return The command to run the SysId routine.
         */
        public Command sysIdQuasistaticRotation(SysIdRoutine.Direction direction) {
            return sysIdRotation.quasistatic(direction);
        }

        /**
         * Gets a command to run the SysId rotation routine in the dynamic mode.
         * @param direction The direction to run the SysId routine.
         * @return The command to run the SysId routine.
         */
        public Command sysIdDynamicRotation(SysIdRoutine.Direction direction) {
            return sysIdRotation.dynamic(direction);
        }

        public DoubleProperty getDriveToWaypointsSpeed() {
            return driveToWaypointsSpeed;
        }

        public DoubleProperty getDriveToWaypointsDurationPerPoint() {
            return driveToWaypointsDurationPerPoint;
        }
    }