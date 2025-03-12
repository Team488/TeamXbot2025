package competition.subsystems.drive.commands;

import competition.subsystems.coral_scorer.CoralScorerSubsystem;
import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.pose.Landmarks;
import competition.subsystems.pose.PoseSubsystem;
import edu.wpi.first.wpilibj.DriverStation;
import xbot.common.command.BaseCommand;
import xbot.common.controls.sensors.XTimer;
import xbot.common.math.XYPair;
import xbot.common.properties.DoubleProperty;
import xbot.common.properties.PropertyFactory;

import javax.inject.Inject;

public class ShoveCoralStationCommand extends BaseCommand {

    final DriveSubsystem drive;
    final PoseSubsystem pose;
    final CoralScorerSubsystem coralScorer;
    PropertyFactory pf;

    // Properties
    final DoubleProperty shovePower;

    double shoveAngleDegreesRequested;
    double shoveAngleActual;


    @Inject
    public ShoveCoralStationCommand(DriveSubsystem drive, PoseSubsystem pose, PropertyFactory pf, CoralScorerSubsystem coralScorer) {
        this.drive = drive;
        this.pose = pose;
        this.coralScorer = coralScorer;

        pf.setPrefix(this);
        shovePower = pf.createPersistentProperty("ShovePower", 0.25);

        this.addRequirements(drive);
    }

    public void setShoveAngle(Landmarks.CoralStation coralStation) {
        if (coralStation == Landmarks.CoralStation.LEFT) {
            shoveAngleDegreesRequested = Landmarks.BlueLeftCoralStationMid.getRotation().getDegrees();
        }
        else {
            shoveAngleDegreesRequested = Landmarks.BlueRightCoralStationMid.getRotation().getDegrees();
        }
        // We want to shove in, default headings are "out"
        shoveAngleDegreesRequested += 180;
    }

    @Override
    public void initialize() {
        log.info("Initializing");
        shoveAngleActual = shoveAngleDegreesRequested;
        if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red) {
            shoveAngleActual = shoveAngleDegreesRequested + 180;
        }
    }

    @Override
    public void execute() {
        drive.fieldOrientedDrive(XYPair.fromPolar(shoveAngleActual, shovePower.get()), 0, pose.getCurrentHeading().getDegrees(), true);
        aKitLog.record("ShoveAngleDegrees", shoveAngleActual);
    }

    @Override
    public boolean isFinished() {
        return coralScorer.confidentlyHasCoral();
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }
}
