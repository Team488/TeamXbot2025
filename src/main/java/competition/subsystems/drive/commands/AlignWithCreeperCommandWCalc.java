package competition.subsystems.drive.commands;

import competition.electrical_contract.ElectricalContract;
import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.drive.logic.AlignWithCreeperCalculator;
import competition.subsystems.oracle.ReefCoordinateGenerator;
import competition.subsystems.pose.Cameras;
import competition.subsystems.pose.PoseSubsystem;
import competition.subsystems.vision.AprilTagVisionSubsystemExtended;
import competition.subsystems.vision.CoprocessorCommunicationSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.kobe.xbot.JClient.CachedSubscriber;
import org.kobe.xbot.JClient.XTablesClient;
import xbot.common.command.BaseCommand;
import xbot.common.math.PIDManager;
import xbot.common.math.PIDManager.PIDManagerFactory;
import xbot.common.math.XYPair;
import xbot.common.properties.DoubleProperty;
import xbot.common.properties.PropertyFactory;
import xbot.common.properties.StringProperty;
import xbot.common.subsystems.drive.control_logic.HeadingModule;
import competition.subsystems.drive.logic.AlignWithCreeperCalculator;
import javax.inject.Inject;


public class AlignWithCreeperCommandWCalc extends BaseCommand {
    private DriveSubsystem drive;
    private AlignWithCreeperCalculator calculator;
    private Cameras camera;

    @Inject
    public AlignWithCreeperCommandWCalc(DriveSubsystem drive, AlignWithCreeperCalculator calculator) {
        this.drive = drive;
        this.addRequirements(drive);
        this.calculator = calculator;
    }

    @Override
    public void initialize() {
        this.calculator.setCamera(this.camera);
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

    public Cameras getCamera() {
        return camera;
    }

    public AlignWithCreeperCommandWCalc setCamera(Cameras camera) {
        this.camera = camera;
        return this;
    }
}
