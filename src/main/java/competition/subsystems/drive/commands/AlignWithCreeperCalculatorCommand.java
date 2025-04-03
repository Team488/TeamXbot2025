package competition.subsystems.drive.commands;

import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.drive.logic.AlignWithCreeperCalculator;
import competition.subsystems.pose.Cameras;
import xbot.common.command.BaseCommand;

import javax.inject.Inject;


public class AlignWithCreeperCalculatorCommand extends BaseCommand {
    private DriveSubsystem drive;
    private AlignWithCreeperCalculator calculator;

    @Inject
    public AlignWithCreeperCalculatorCommand(DriveSubsystem drive, AlignWithCreeperCalculator calculator) {
        this.drive = drive;
        this.addRequirements(drive);
        this.calculator = calculator;
    }

    @Override
    public void initialize() {
        if(!this.calculator.initialize()){
            log.warn("Failed to initialize Calculator.");
            cancel();
        };
    }


    public void execute() {
        if (!this.calculator.executeAlignment()) {
            log.warn("Failed to execute AlignWithCreeperCalculator.");
        }
    }


    @Override
    public boolean isFinished() {
        // us being centered is only valuable if we arent currently moving
        return calculator.isFinished(true);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        if (interrupted) {
            log.warn("Alignment command interrupted before completion.");
        }
        drive.stop();
    }

}
