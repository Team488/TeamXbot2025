package competition.subsystems.drive.commands;

import competition.subsystems.coral_scorer.CoralScorerSubsystem;
import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.pose.Landmarks;
import competition.subsystems.pose.PoseSubsystem;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import xbot.common.command.BaseCommand;
import xbot.common.math.XYPair;
import xbot.common.properties.DoubleProperty;
import xbot.common.properties.PropertyFactory;

import javax.inject.Inject;

import static edu.wpi.first.units.Units.Degrees;

public class ShoveCoralStationCommand extends BaseCommand {

    final DriveSubsystem drive;
    final PoseSubsystem pose;
    final CoralScorerSubsystem coralScorer;

    final DoubleProperty shovePower;

    Angle shoveAngleRequested;
    Angle shoveAngleActual;

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
            shoveAngleRequested = Degrees.of(Landmarks.BlueLeftCoralStationMid.getRotation().getDegrees());
        }
        else {
            shoveAngleRequested = Degrees.of(Landmarks.BlueRightCoralStationMid.getRotation().getDegrees());
        }
        // We want to shove in, default headings are "out"
        shoveAngleRequested = shoveAngleRequested.plus(Degrees.of(180));
    }

    @Override
    public void initialize() {
        log.info("Initializing");
        shoveAngleActual = shoveAngleRequested;
        if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red) {
            shoveAngleActual = shoveAngleRequested.plus(Degrees.of(180));
        }
        aKitLog.record("ShoveAngleDegrees", shoveAngleActual);
    }

    @Override
    public void execute() {
        drive.fieldOrientedDrive(
                XYPair.fromPolar(shoveAngleActual.in(Degrees), shovePower.get()),
                0,
                pose.getCurrentHeading().getDegrees(),
                true
        );
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
