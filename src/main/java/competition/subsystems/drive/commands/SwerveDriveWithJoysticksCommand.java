package competition.subsystems.drive.commands;

import competition.operator_interface.OperatorInterface;
import competition.subsystems.drive.DriveSubsystem;
import edu.wpi.first.wpilibj.DriverStation;
import xbot.common.logic.HumanVsMachineDecider;
import xbot.common.logic.HumanVsMachineDecider.HumanVsMachineDeciderFactory;
import xbot.common.subsystems.drive.swerve.SwerveSuggestedRotation;
import xbot.common.subsystems.drive.swerve.SwerveDriveRotationAdvisor;
import competition.subsystems.pose.PoseSubsystem;
import xbot.common.command.BaseCommand;
import xbot.common.math.MathUtils;
import xbot.common.math.XYPair;
import xbot.common.properties.DoubleProperty;
import xbot.common.properties.Property;
import xbot.common.properties.PropertyFactory;
import xbot.common.subsystems.drive.control_logic.HeadingModule;
import xbot.common.subsystems.drive.control_logic.HeadingModule.HeadingModuleFactory;

import javax.inject.Inject;

public class SwerveDriveWithJoysticksCommand extends BaseCommand {

    OperatorInterface oi;
    DriveSubsystem drive;
    PoseSubsystem pose;
    HeadingModule headingModule;

    DoubleProperty overallDrivingPowerScale;
    DoubleProperty overallTurningPowerScale;

    SwerveDriveRotationAdvisor advisor;
    HumanVsMachineDecider hvmDecider;

    @Inject
    public SwerveDriveWithJoysticksCommand(
            OperatorInterface oi, DriveSubsystem drive, PoseSubsystem pose, PropertyFactory pf,
            HeadingModuleFactory headingModuleFactory, HumanVsMachineDeciderFactory hvmFactory) {
        pf.setPrefix(this);
        this.drive = drive;
        this.pose = pose;
        this.oi = oi;
        this.headingModule = headingModuleFactory.create(drive.getRotateToHeadingPid());
        this.hvmDecider = hvmFactory.create(pf.getPrefix());
        this.advisor = new SwerveDriveRotationAdvisor(pose, drive, pf, hvmDecider, 0.001);
        pf.setDefaultLevel(Property.PropertyLevel.Important);
        pf.setPrefix(this);
        this.overallDrivingPowerScale = pf.createPersistentProperty("DrivingPowerScale", 1.0);
        this.overallTurningPowerScale = pf.createPersistentProperty("TurningPowerScale", 1.0);
        this.addRequirements(drive);
    }

    @Override
    public void initialize() {
        log.info("Initializing");
        advisor.resetDecider();
        drive.setDesiredHeading(pose.getCurrentHeading().getDegrees());
    }

    @Override
    public void execute() {
        // Get raw human translate and rotation intents
        XYPair translationIntent = getRawHumanTranslationIntent();
        double rawRotationIntent = getRawHumanRotationIntent();

        // Process the translation intent
        translationIntent = getSuggestedTranslationIntent(translationIntent);

        // Checks snapping to side or other rotation features to get suggested intent
        double rotationIntent = getSuggestedRotationIntent(rawRotationIntent);

        if (!drive.isUnlockFullDrivePowerActive()) {
            translationIntent = translationIntent.scale(overallDrivingPowerScale.get());
            rotationIntent *= overallTurningPowerScale.get();
        }

        // Field oriented drive will process the actual swerve movements for us
        drive.fieldOrientedDrive(
                translationIntent,
                rotationIntent,
                pose.getCurrentHeading().getDegrees(),
                new XYPair(0,0)
        );
    }

    private XYPair getRawHumanTranslationIntent() {
        XYPair translationIntent = new XYPair(
                oi.driverGamepad.getLeftVector().getX(),
                oi.driverGamepad.getLeftVector().getY());

        if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red) {
            translationIntent.rotate(180);
        }

        // We have to rotate -90 degrees to fix some alignment issues
        return translationIntent.rotate(-90);
    }

    private double getRawHumanRotationIntent() {
        // Deadband is to prevent buggy joysticks/triggers
        double rotateLeftIntent = MathUtils.deadband(oi.driverGamepad.getLeftTrigger(), 0.05, (x) -> MathUtils.exponentAndRetainSign(x, 2));
        double rotateRightIntent = MathUtils.deadband(oi.driverGamepad.getRightTrigger(), 0.05, (x) -> MathUtils.exponentAndRetainSign(x, 2));

        // Merge the two trigger values together in case of conflicts
        // Rotate left = positive, right = negative
        return rotateLeftIntent - rotateRightIntent;
    }

    public double getSuggestedRotationIntent(double triggerRotateIntent) {
        // Checks the right joystick input to see if we want to snap to a certain side
        // Apparently, we need to invert the x input here as it has been inverted for other commands already
        // And of course, we must rotate -90 (similar to how we got raw translation) for default alignment
        XYPair joystickInput = new XYPair(-oi.driverGamepad.getRightVector().getX(), oi.driverGamepad.getRightVector().getY()).rotate(-90);

        SwerveSuggestedRotation suggested = advisor.getSuggestedRotationValue(joystickInput, triggerRotateIntent);
        return processSuggestedRotationValueIntoPower(suggested);
    }

    private XYPair getSuggestedTranslationIntent(XYPair intent) {

        aKitLog.record("RawTranslationIntent", intent);

        // The intent has no deadband applied yet. Check to see if we are below deadband; if so, just return 0.
        double magnitude = intent.getMagnitude();
        if (intent.getMagnitude() < oi.getDriverGamepadTypicalDeadband()) {
            return new XYPair(0, 0);
        }

        aKitLog.record("Magnitude", magnitude);

        // If we're here, then the joystick is deflected enough we need to take action. Take the magnitude and
        // apply the advanced deadband function to it.
        double scaledMagnitude = MathUtils.deadband(
                magnitude,
                oi.getDriverGamepadTypicalDeadband(),
                (x) -> MathUtils.exponentAndRetainSign(x, 3));

        aKitLog.record("ScaledMagnitude", scaledMagnitude);

        // Now that we have the scaled magnitude, we need to apply it to the direction of the intent.
        // This is done by taking the unit vector of the intent and multiplying it by the scaled magnitude.
        intent = XYPair.fromPolar(intent.getAngle(), scaledMagnitude);

        aKitLog.record("ScaledTranslationIntent", intent);


        if (!drive.isUnlockFullDrivePowerActive()) {
            // Scale translationIntent if precision modes active, values from XBot2024 repository
            if (drive.isExtremePrecisionTranslationActive()) {
                intent = intent.scale(0.15);
            } else if (drive.isPrecisionTranslationActive()) {
                intent = intent.scale(0.50);
            }
        }
        return intent;
    }

    private double processSuggestedRotationValueIntoPower(SwerveSuggestedRotation suggested) {
        return switch (suggested.type) {
            case DesiredHeading -> {
                yield headingModule.calculateHeadingPower(suggested.value);
            }
            case HumanControlHeadingPower -> {
                if (drive.isPrecisionRotationActive()) {
                    yield suggested.value *= 0.25;
                }
                yield suggested.value;
            }
        };
    }
}