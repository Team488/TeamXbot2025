
package competition;

import au.grapplerobotics.CanBridge;
import competition.electrical_contract.ElectricalContract;
import competition.electrical_contract.UnitTestContract2025;
import competition.injection.components.BaseRobotComponent;
import competition.injection.components.DaggerRobotComponent;
import competition.injection.components.DaggerRobotComponent2023;
import competition.injection.components.DaggerRobotComponent2024;
import competition.injection.components.DaggerRoboxComponent;
import competition.injection.components.DaggerSimulationComponent;
import competition.operator_interface.OperatorInterface;
import competition.simulation.BaseSimulator;
import competition.subsystems.pose.PoseSubsystem;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Preferences;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import xbot.common.command.BaseRobot;
import xbot.common.command.XScheduler;
import xbot.common.math.FieldPose;
import xbot.common.subsystems.pose.BasePoseSubsystem;

import java.util.concurrent.CountDownLatch;

public class Robot extends BaseRobot {
    Logger log = LogManager.getLogger(Robot.class);

    final CountDownLatch reachedDisabledInit = new CountDownLatch(1);
    final CountDownLatch reachedEndOfLoop = new CountDownLatch(5);

    BaseSimulator simulator;
    ElectricalContract simulatorContract = new UnitTestContract2025();
    OperatorInterface oi;

    Robot() {
        // We currently can't keep up with 0.02s loop times, and the error reporting about loop
        // overruns ironically makes the problem worse. For now, we're going to set the loop time
        // to 0.04s to give us some breathing room and figure out some optimizations to bring us
        // back down to 0.02s.
        super(0.04);
    }

    @Override
    protected void initializeSystems() {
        super.initializeSystems();
        getInjectorComponent().subsystemDefaultCommandMap();
        getInjectorComponent().operatorCommandMap();
        getInjectorComponent().swerveDefaultCommandMap();
        getInjectorComponent().superstructureMechanismSubsystem();
        getInjectorComponent().oracleSubsystem();
        getInjectorComponent().lightSubsystem();
        oi = getInjectorComponent().operatorInterface();

        if (BaseRobot.isSimulation()) {
            simulator = getInjectorComponent().simulator();
        }


        dataFrameRefreshables.add(getInjectorComponent().driveSubsystem());
        dataFrameRefreshables.add(getInjectorComponent().poseSubsystem());
        dataFrameRefreshables.add(getInjectorComponent().coprocessorCommunicationSubsystem());
        dataFrameRefreshables.add(getInjectorComponent().aprilTagVisionSubsystemExtended());
        dataFrameRefreshables.add(getInjectorComponent().armPivotSubsystem());
        dataFrameRefreshables.add(getInjectorComponent().elevatorSubsystem());
        dataFrameRefreshables.add(getInjectorComponent().coralScorerSubsystem());
        dataFrameRefreshables.add(getInjectorComponent().algaeCollectionSubsystem());
        dataFrameRefreshables.add(getInjectorComponent().algaeArmSubsystem());

        // Not needed unless we are actively configuring the LaserCAN.
        //CanBridge.runTCP();
    }

    protected BaseRobotComponent createDaggerComponent() {
        if (BaseRobot.isReal()) {

            if (!Preferences.containsKey("ContractToUse")) {
                log.error("No contract set in Preferences! This is likely unexpected.");
                log.info("Count of keys in the Preferences system: {}", Preferences.getKeys().size());
            }

            String chosenContract = Preferences.getString("ContractToUse", "Competition");

            switch (chosenContract) {
                case "2023":
                    log.info("Using 2023 contract");
                    return DaggerRobotComponent2023.create();
                case "2024":
                    log.info("Using 2024 contract");
                    return DaggerRobotComponent2024.create();
                case "Robox":
                    log.info("Using Robox contract");
                    return DaggerRoboxComponent.create();
                default:
                    // Moved setting the default contract to here; there was some bug where
                    // other chassis were being set to "Competition" when they shouldn't have been.
                    // Root cause is unknown, so far now only setting this after checking for any other value.
                    if (!Preferences.containsKey("ContractToUse")) {
                        Preferences.setString("ContractToUse", "Competition");
                    }
                    log.info("Using Competition contract");
                    // In all other cases, return the competition component.
                    return DaggerRobotComponent.create();
            }
        } else {
            log.warn("Using simulation contract");
            return DaggerSimulationComponent
                    .builder()
                    .electricalContract(simulatorContract)
                    .build();
        }
    }

    public BaseRobotComponent getInjectorComponent() {
        return (BaseRobotComponent)super.getInjectorComponent();
    }

    @Override
    public void disabledInit() {
        super.disabledInit();
        reachedDisabledInit.countDown();
        var poseSub = getInjectorComponent().poseSubsystem();
        poseSub.getResetTranslationToVisionEstimateCommand().schedule();
    }

    @Override
    public void simulationInit() {
        super.simulationInit();
        // Automatically enables the robot; remove this line of code if you want the robot
        // to start in a disabled state (as it would on the field). However, this does save you the 
        // hassle of navigating to the DS window and re-enabling the simulated robot.

        //webots.setFieldPoseOffset(getFieldOrigin());
        // Line below isn't working, still sets to the red alliance... need to investigate further.
        // DriverStationSim.setAllianceStationId(AllianceStationID.Blue2);
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

    @Override
    protected void loopFunc() {
        super.loopFunc();
        reachedEndOfLoop.countDown();
    }

    public XScheduler getScheduler() {
        return xScheduler;
    }

    @Override
    public void sharedPeriodic() {
        super.sharedPeriodic();
        if(oi != null) {
            double propertyStart = getPerformanceTimestampInMs();
            oi.periodic();
            double propertyEnd = getPerformanceTimestampInMs();
            org.littletonrobotics.junction.Logger.recordOutput("OI Periodic ms", propertyEnd - propertyStart);
        }
    }
}
