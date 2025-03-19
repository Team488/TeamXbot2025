

package competition.subsystems.lights;

import javax.inject.Inject;
import javax.inject.Singleton;

import competition.electrical_contract.ElectricalContract;
import competition.subsystems.coral_arm.CoralArmSubsystem;
import competition.subsystems.coral_scorer.CoralScorerSubsystem;
import competition.subsystems.drive.logic.AlignCameraToAprilTagCalculator;
import competition.subsystems.elevator.ElevatorSubsystem;
import competition.subsystems.vision.AprilTagVisionSubsystemExtended;
import edu.wpi.first.wpilibj.DriverStation;
import xbot.common.command.BaseSubsystem;
import xbot.common.controls.actuators.XDigitalOutput;
import xbot.common.controls.actuators.XDigitalOutput.XDigitalOutputFactory;
import xbot.common.controls.sensors.XTimer;
import xbot.common.subsystems.autonomous.AutonomousCommandSelector;

import java.sql.Time;
import java.util.Objects;


@Singleton
public class LightSubsystem extends BaseSubsystem {
    // based on the number of bits we have, this is the highest number we can send
    static final int numBits = 4;
    static final int maxValue = (int)(Math.pow(2, numBits) - 1);

    final AutonomousCommandSelector autonomousCommandSelector;
    final CoralScorerSubsystem coralScorerSubsystem;
    final CoralArmSubsystem coralArmSubsystem;
    final ElevatorSubsystem elevatorSubsystem;
    final AprilTagVisionSubsystemExtended visionSubsystem;
    private AlignCameraToAprilTagCalculator.Activity activity;
    private int targetCameraID;
    private boolean recentlyAligned;
    private double lastStateTime = -1;
    LightsStateMessage state = LightsStateMessage.NoCode;
    DIOInt dioInt;

    public enum LightsStateMessage{
        // we never send NoCode, it's implicit when the robot is off
        // and all of the DIOs float high
        NoCode(15),
        RobotDisabledDefault(1),
        RobotDisabledAuto(2),
        RobotEnabled(3),
        CoralPresent(4),
        RequestCoralFromHuman(5),
        Victory(6), 
        //ReadyToScore(7),
        DisabledCameraUnavailable(8),
        TargetCameraUnavailable(9),
        CurrentlyAligning(10),
        FinishedAligning(11),
        CreeperActive(12),
        NoCoralPresent(13);

    
    
        // CoralReset(101),
        // AlgaeDrop(7),
        // AlgaeGrab(8),
        // AlgaePush(9),
        // Auto1(10),
        // Auto2(11),
        // Auto3(12),
        // HangDeep(13),
        // HangShallow(14),
        // ElevatorFinish(15),
        // ElevatorRaise2(16),
        // ElevatorRaise3(17),
        // ElevatorRaise4(18),
        // StartPosition(20);

        LightsStateMessage(final int value) {
            if(value > maxValue || value < 0) {
                // it should be okay to have this throw because it will happen immediately on robot startup
                // so we'll see failures here in CI before deploying to the robot. 
                // Getting the RobotAssertionManager in here was proving tricky
                System.out.println("Values must be between 0 and " + maxValue + " inclusive. Got " + value + " instead. Will always return 0 for safety.");
            }
            this.value = value;
        }

        private int value;
        public int getValue() {
            if (value < 0 || value > maxValue) {
                return 0;
            }
            return value;
        }

        public static LightsStateMessage getStringValueFromInt(int i) {
            for (LightsStateMessage states : LightsStateMessage.values()) {
                if (states.getValue() == i) {
                    return states;
                }
            }
           return LightsStateMessage.NoCode;
        }
    }

    @Inject
    public LightSubsystem(XDigitalOutputFactory digitalOutputFactory,
                          ElectricalContract contract,
                          AutonomousCommandSelector autonomousCommandSelector,
                          CoralScorerSubsystem coralScorerSubsystem,
                          CoralArmSubsystem coralArmSubsystem,
                          ElevatorSubsystem elevatorSubsystem,
                          AprilTagVisionSubsystemExtended visionSubsystem) {
        this.autonomousCommandSelector = autonomousCommandSelector;
        this.coralScorerSubsystem = coralScorerSubsystem;
        this.coralArmSubsystem = coralArmSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;
        this.visionSubsystem = visionSubsystem;
        XDigitalOutput[] dios = {
            digitalOutputFactory.create(contract.getLightsDio0().channel),
            digitalOutputFactory.create(contract.getLightsDio1().channel), 
            digitalOutputFactory.create(contract.getLightsDio2().channel), 
            digitalOutputFactory.create(contract.getLightsDio3().channel)};
        this.dioInt = new DIOInt(dios);
    }

    public LightsStateMessage getCurrentState() {
        boolean dsEnabled = DriverStation.isEnabled();
        LightsStateMessage currentState;

        // Needs to implement vision as well
        // Not sure about if the way we are checking the shooter is correct (and collector)
        if (!dsEnabled) {
            currentState = LightsStateMessage.RobotDisabledDefault;
            if (DriverStation.getMatchTime() > 230 && DriverStation.isTeleop()) {
                currentState = LightsStateMessage.Victory;
            }
            if (!visionSubsystem.areAllCamerasConnected()) {
                currentState = LightsStateMessage.DisabledCameraUnavailable;
            }
            if (!Objects.equals(autonomousCommandSelector.getProgramName(), "EmergencyAutonomousCommand")) {
                currentState = LightsStateMessage.RobotDisabledAuto;
            }
        } else {
            if (activity == AlignCameraToAprilTagCalculator.Activity.TerminalApproach) {
                if (!visionSubsystem.isCameraConnected(targetCameraID)) {
                    currentState = LightsStateMessage.TargetCameraUnavailable;
                } else {
                    currentState = LightsStateMessage.CurrentlyAligning;
                    recentlyAligned = true;
                }
            } else if (activity == AlignCameraToAprilTagCalculator.Activity.Complete && recentlyAligned) {
                currentState = LightsStateMessage.FinishedAligning;

                if (lastStateTime == -1) {
                    lastStateTime = XTimer.getFPGATimestamp();
                }

                if (XTimer.getFPGATimestamp() - lastStateTime >= 2) {
                    recentlyAligned = false;
                    lastStateTime = -1;
                }
            } else if (coralScorerSubsystem.getCoralScorerState() == CoralScorerSubsystem.CoralScorerState.INTAKING) {
                currentState = LightsStateMessage.RequestCoralFromHuman;
            } else if (!coralScorerSubsystem.confidentlyHasCoral()) {
                currentState = LightsStateMessage.NoCoralPresent;
            } else if (coralScorerSubsystem.confidentlyHasCoral()) {
                currentState = LightsStateMessage.CoralPresent;
            } else {
                currentState = LightsStateMessage.RobotEnabled;
            }
        }
        return currentState;
    }
    public void sendState(LightsStateMessage state) {
        dioInt.setDIOInt(state.getValue());
    }

    public LightsStateMessage getState() {
        return state;
    }

    public void updateFromCalculator(AlignCameraToAprilTagCalculator.Activity activity, int targetCameraID) {
        this.activity = activity;
        this.targetCameraID = targetCameraID;
    }

    public void updateTargetCameraID(int targetCameraID) {
        this.targetCameraID = targetCameraID;
    }

    @Override
    public void periodic() {
        this.state = getCurrentState();
        sendState(state);

        aKitLog.record("LightState", state.toString());
    }
    
    protected class DIOInt {
        private XDigitalOutput[] dios;
        private static int numDios;

        public DIOInt(XDigitalOutput[] dios) {
            this.dios = dios;
            numDios = dios.length;
        }

        /**
         * Convert an integer to a boolean array representing the bits of the integer.
         * The leftmost bit in the result is the least significant bit of the integer.
         * This was chosen so we could add new bits onto the end of the array easily without changing
         * how earlier numbers were represented.
         * Eg: 
         * 0 -> [false, false, false, false]
         * 1 -> [true, false, false, false]
         * 14 -> [false, true, true, true]
         * 15 -> [true, true, true, true]
         */
        private static boolean[] convertIntToBits(int value) {
            boolean[] bits = new boolean[numDios];
            for(int i = 0; i < numDios; i++) {
                bits[i] = (value & (1 << i)) != 0;
            }
            return bits;
        }

        public void setDIOInt(int num) {
            boolean[] bitsToSet = convertIntToBits(num);

            for(int i = 0; i < numDios; i++) {
                dios[i].set(bitsToSet[i]);
            }
        }

        public int getDIOInt() {
            int value = 0;
            
            for (int i = 0; i < numDios; i++) {
                value += dios[i].get() ? (1L << i) : 0L;
            }

            return value;
        }
    }
}