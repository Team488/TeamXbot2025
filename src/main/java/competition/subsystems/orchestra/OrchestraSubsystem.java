package competition.subsystems.orchestra;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;
import competition.subsystems.drive.DriveSubsystem;
import org.apache.logging.log4j.LogManager;
import org.littletonrobotics.junction.Logger;
import xbot.common.command.BaseSubsystem;
import xbot.common.controls.actuators.XCANMotorController;
import xbot.common.controls.actuators.wpi_adapters.CANTalonFxWpiAdapter;
import xbot.common.subsystems.drive.swerve.SwerveModuleSubsystem;

import javax.inject.Inject;
import java.util.ArrayList;
import java.util.List;

public class OrchestraSubsystem extends BaseSubsystem {
    private final org.apache.logging.log4j.Logger log = LogManager.getLogger(OrchestraSubsystem.class);
    private final Orchestra orchestra;
    private final List<TalonFX> motorControllers;
    private String currentSong;

    public static class Songs {
        public static final String NOKIA_TUNE = "nokia_tune.chrp";
        public static final String RICKROLL = "rickroll.chrp";
    }

    @Inject
    public OrchestraSubsystem(DriveSubsystem drive) {
        this.orchestra = new Orchestra();
        this.motorControllers = new ArrayList<>();

        List<XCANMotorController> motors = new ArrayList<>();
        for (SwerveModuleSubsystem module : new SwerveModuleSubsystem[] {
                drive.getFrontLeftSwerveModuleSubsystem(),
                drive.getFrontRightSwerveModuleSubsystem(),
                drive.getRearLeftSwerveModuleSubsystem(),
                drive.getRearRightSwerveModuleSubsystem()
        }) {
            motors.addAll(getSwervePodMotorControllers(module));
        }

        for (var motor : motors) {
            if (motor instanceof CANTalonFxWpiAdapter adapter) {
                TalonFX talon = adapter.getInternalTalonFx();
                adapter.setEnableMusicDuringDisable(true);
                orchestra.addInstrument(talon);
                motorControllers.add(talon);
            }
        }

        log.info("Orchestra created with {} players", motorControllers.size());
    }

    public void loadMusicSelection(String song)
    {
        var status = this.orchestra.loadMusic(song);
        if (!status.isOK()) {
            log.error("Failed to load music: {}", status.getDescription());
        } else {
            this.currentSong = song;
        }
    }

    public boolean isPlaying() {
        return this.orchestra.isPlaying();
    }

    public boolean tryPlay() {
        if (this.orchestra.isPlaying()) {
            return false;
        }

        var status = this.orchestra.play();
        if (!status.isOK()) {
            log.error("Failed to play music: {}", status.getDescription());
        }
        return status.isOK();
    }

    public boolean tryPause() {
        if (!this.orchestra.isPlaying()) {
            return false;
        }

        var status = this.orchestra.pause();
        return status.isOK();
    }

    public boolean tryStop() {
        if (!this.orchestra.isPlaying()) {
            return false;
        }

        var status = this.orchestra.stop();
        return status.isOK();
    }

    @Override
    public void periodic() {
        Logger.recordOutput(this.getPrefix() + "/IsPlaying", this.orchestra.isPlaying());
        Logger.recordOutput(this.getPrefix() + "/CurrentSong", this.currentSong);
    }

    private List<XCANMotorController> getSwervePodMotorControllers(SwerveModuleSubsystem module) {
        List<XCANMotorController> controllers = new ArrayList<>();
        module.getDriveSubsystem().getMotorController().ifPresent(controllers::add);
        module.getSteeringSubsystem().getMotorController().ifPresent(controllers::add);
        return controllers;
    }
}
