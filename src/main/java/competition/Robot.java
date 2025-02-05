
package competition;

import au.grapplerobotics.CanBridge;
import competition.injection.components.BaseRobotComponent;
import competition.injection.components.DaggerRobotComponent;
import competition.injection.components.DaggerRobotComponent2023;
import competition.injection.components.DaggerRobotComponent2024;
import competition.injection.components.DaggerRoboxComponent;
import competition.injection.components.DaggerSimulationComponent;
import competition.simulation.BaseSimulator;
import competition.subsystems.drive.DriveSubsystem;
import competition.subsystems.pose.PoseSubsystem;
import edu.wpi.first.wpilibj.Preferences;
import xbot.common.command.BaseRobot;
import xbot.common.math.FieldPose;
import xbot.common.subsystems.pose.BasePoseSubsystem;

public class Robot extends BaseRobot {

    BaseSimulator simulator;

    @Override
    protected void initializeSystems() {
        super.initializeSystems();
        getInjectorComponent().subsystemDefaultCommandMap();
        getInjectorComponent().operatorCommandMap();
        getInjectorComponent().swerveDefaultCommandMap();
        getInjectorComponent().superstructureMechanismSubsystem();
        getInjectorComponent().oracleSubsystem();

        if (BaseRobot.isSimulation()) {
            simulator = getInjectorComponent().simulator();
        }

        dataFrameRefreshables.add((DriveSubsystem)getInjectorComponent().driveSubsystem());
        dataFrameRefreshables.add(getInjectorComponent().poseSubsystem());
        dataFrameRefreshables.add(getInjectorComponent().coprocessorCommunicationSubsystem());
        dataFrameRefreshables.add(getInjectorComponent().aprilTagVisionSubsystemExtended());
        dataFrameRefreshables.add(getInjectorComponent().armPivotSubsystem());
        dataFrameRefreshables.add(getInjectorComponent().elevatorSubsystem());
        dataFrameRefreshables.add(getInjectorComponent().coralScorerSubsystem());

        CanBridge.runTCP();
    }

    protected BaseRobotComponent createDaggerComponent() {
        if (BaseRobot.isReal()) {

            String chosenContract = Preferences.getString("ContractToUse", "Competition");

            switch (chosenContract) {
                case "2023":
                    System.out.println("Using 2023 contract");
                    return DaggerRobotComponent2023.create();
                case "2024":
                    System.out.println("Using 2024 contract");
                    return DaggerRobotComponent2024.create();
                case "Robox":
                    System.out.println("Using Robox contract");
                    return DaggerRoboxComponent.create();
                default:
                    // Moved setting the default contract to here; there was some bug where
                    // other chassis were being set to "Competition" when they shouldn't have been.
                    // Root cause is unknown, so far now only setting this after checking for any other value.
                    if (!Preferences.containsKey("ContractToUse")) {
                        Preferences.setString("ContractToUse", "Competition");
                    }
                    System.out.println("Using Competition contract");
                    // In all other cases, return the competition component.
                    return DaggerRobotComponent.create();
            }
        } else {
            return DaggerSimulationComponent.create();
        }
    }

    public BaseRobotComponent getInjectorComponent() {
        return (BaseRobotComponent)super.getInjectorComponent();
    }

    @Override
    public void simulationInit() {
        super.simulationInit();
        // Automatically enables the robot; remove this line of code if you want the robot
        // to start in a disabled state (as it would on the field). However, this does save you the 
        // hassle of navigating to the DS window and re-enabling the simulated robot.

        //webots.setFieldPoseOffset(getFieldOrigin());
    }

    private FieldPose getFieldOrigin() {
        // Modify this to whatever the simulator coordinates are for the "FRC origin" of the field.
        // From a birds-eye view where your alliance station is at the bottom, this is the bottom-left corner
        // of the field.
        return new FieldPose(
            -2.33*PoseSubsystem.INCHES_IN_A_METER, 
            -4.58*PoseSubsystem.INCHES_IN_A_METER, 
            BasePoseSubsystem.FACING_TOWARDS_DRIVERS
            );
    }

    @Override
    public void simulationPeriodic() {
        super.simulationPeriodic();

        if (simulator != null) {
            simulator.update();
        }
    }
}
