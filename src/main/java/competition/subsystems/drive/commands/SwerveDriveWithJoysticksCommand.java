package competition.subsystems.drive.commands;

import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.drive.logic.ManualSwerveDriveLogic;
import edu.wpi.first.wpilibj.DriverStation;
import xbot.common.subsystems.drive.swerve.SwerveSuggestedRotation;
import xbot.common.command.BaseCommand;
import xbot.common.math.MathUtils;
import xbot.common.math.XYPair;

import javax.inject.Inject;

public class SwerveDriveWithJoysticksCommand extends BaseCommand {

    DriveSubsystem drive;
    ManualSwerveDriveLogic logic;

    @Inject
    public SwerveDriveWithJoysticksCommand( DriveSubsystem drive,
            ManualSwerveDriveLogic.ManualSwerveDriveLogicFactory manualSwerveDriveLogicFactory) {
        this.drive = drive;
        logic = manualSwerveDriveLogicFactory.create();
        this.addRequirements(drive);
    }

    @Override
    public void initialize() {
        log.info("Initializing");
        logic.initialize();
    }

    @Override
    public void execute() {
        var advice = logic.getDriveAdvice();
        drive.fieldOrientedDrive(
                advice.translation(),
                advice.rotationIntent(),
                advice.currentHeading(),
                advice.centerOfRotationInches()
        );
    }
}