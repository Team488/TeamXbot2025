package competition.subsystems.drive.commands;

import competition.electrical_contract.ElectricalContract;
import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.oracle.ReefCoordinateGenerator;
import competition.subsystems.pose.Cameras;
import competition.subsystems.pose.PoseSubsystem;
import competition.subsystems.vision.AprilTagVisionSubsystemExtended;
import competition.subsystems.vision.CoprocessorCommunicationSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

import org.kobe.xbot.JClient.CachedSubscriber;
import org.kobe.xbot.JClient.XTablesClient;
import xbot.common.command.BaseCommand;
import xbot.common.math.XYPair;
import xbot.common.properties.DoubleProperty;
import xbot.common.properties.PropertyFactory;
import xbot.common.properties.StringProperty;
import xbot.common.subsystems.drive.control_logic.HeadingModule;

import javax.inject.Inject;
import javax.inject.Provider;

/**
 * Command to align the robot with a creeper target using vision-based error
 * measurements. <p> This command retrieves error values (in pixels) from the
 * vision system via the coprocessor, normalizes the error relative to the
 * camera resolution, applies a gain factor to scale the drive power
 * appropriately, and then moves the robot to align with the target.
 * </p>
 */
public class AlignWithCreeperCommandFactory{
    Provider<AlignWithCreeperCommand> alignWithCreeperCommandProvider;
    
    @Inject
    public AlignWithCreeperCommandFactory(Provider<AlignWithCreeperCommand> alignWithCreeperCommandProvider) {
        this.alignWithCreeperCommandProvider = alignWithCreeperCommandProvider;
    }

    public Command create(Cameras camera) {
        AlignWithCreeperCommand alignWithCreeperCommand = alignWithCreeperCommandProvider.get();
        alignWithCreeperCommand.setCamera(camera);
        return alignWithCreeperCommand;
    }
}
