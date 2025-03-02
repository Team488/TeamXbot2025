package competition.subsystems.orchestra.commands;

import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.orchestra.OrchestraSubsystem;
import dagger.assisted.Assisted;
import dagger.assisted.AssistedFactory;
import dagger.assisted.AssistedInject;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import xbot.common.command.BaseCommand;
import xbot.common.subsystems.drive.swerve.SwerveModuleSubsystem;

public class PlayMusicCommand extends BaseCommand {
    private final Logger log = LogManager.getLogger(PlayMusicCommand.class);

    private final OrchestraSubsystem orchestraSubsystem;
    private final String song;
    private boolean hasStarted = false;

    @AssistedFactory
    public abstract static class Factory {
        public abstract PlayMusicCommand create(@Assisted String song);
    }

    @AssistedInject
    protected PlayMusicCommand(OrchestraSubsystem orchestraSubsystem, DriveSubsystem drive, @Assisted String song) {
        this.orchestraSubsystem = orchestraSubsystem;
        this.song = song;

        addRequirements(orchestraSubsystem);
        for (var swervePod : new SwerveModuleSubsystem[] {
                drive.getFrontLeftSwerveModuleSubsystem(),
                drive.getFrontRightSwerveModuleSubsystem(),
                drive.getRearLeftSwerveModuleSubsystem(),
                drive.getRearRightSwerveModuleSubsystem()
        }) {
            addRequirements(swervePod.getDriveSubsystem(), swervePod.getSteeringSubsystem());
        }
    }

    @Override
    public void initialize() {
        log.info("Playing song: {}", song);
        orchestraSubsystem.loadMusicSelection(song);
        hasStarted = orchestraSubsystem.tryPlay();
    }

    @Override
    public void execute() {
        if (!hasStarted) {
            hasStarted = orchestraSubsystem.tryPlay();
        }
    }

    @Override
    public void end(boolean interrupted) {
        orchestraSubsystem.tryStop();
    }

    @Override
    public boolean isFinished() {
        var isFinished = hasStarted && !orchestraSubsystem.isPlaying();
        if (isFinished) {
            log.info("Done playing music");
        }
        return isFinished;
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
