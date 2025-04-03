package competition.subsystems.drive.commands;

import competition.subsystems.pose.Cameras;
import edu.wpi.first.wpilibj2.command.Command;

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
public class AlignWithCreeperCalculatorCommandFactory {
    Provider<AlignWithCreeperCalculatorCommand> alignWithCreeperWCalcCommandProvider;

    @Inject
    public AlignWithCreeperCalculatorCommandFactory(Provider<AlignWithCreeperCalculatorCommand> alignWithCreeperWCalcCommandProvider) {
        this.alignWithCreeperWCalcCommandProvider = alignWithCreeperWCalcCommandProvider;
    }

    public Command create() {
        AlignWithCreeperCalculatorCommand alignWithCreeperCommand = alignWithCreeperWCalcCommandProvider.get();
        return alignWithCreeperCommand;
    }
}
