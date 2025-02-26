

package competition.subsystems.lights;

import javax.inject.Inject;
import javax.inject.Singleton;

import competition.electrical_contract.ElectricalContract;
import competition.subsystems.coral_scorer.CoralScorerSubsystem;
import edu.wpi.first.wpilibj.DriverStation;
import xbot.common.command.BaseSubsystem;
import xbot.common.controls.actuators.XDigitalOutput.XDigitalOutputFactory;
import xbot.common.subsystems.autonomous.AutonomousCommandSelector;


@Singleton
public class LightSubsystem extends BaseSubsystem {
    // based on the number of bits we have, this is the highest number we can send
    static final int numBits = 4;
    static final int maxValue = (int)(Math.pow(2, numBits) - 1);

    final AutonomousCommandSelector autonomousCommandSelector;
    final CoralScorerSubsystem coralScorerSubsystem;

    LightsStateMessage state = LightsStateMessage.NoCode;

    public enum LightsStateMessage{
        // we never send NoCode, it's implicit when the robot is off
        // and all of the DIOs float high
        NoCode(0),
        RobotDisabled(1),
        RobotEnabled(2),
        CoralPresent(3),
        RequestCoralFromHuman(4),
        ReadyToScore(5),
        CoralReset(6),
        AlgaeDrop(7),
        AlgaeGrab(8),
        AlgaePush(9),
        Auto1(10),
        Auto2(11),
        Auto3(12),
        HangDeep(13),
        HangShallow(14),
        ElevatorFinish(15),
        ElevatorRaise2(16),
        ElevatorRaise3(17),
        ElevatorRaise4(18),
        Victory(19),
        StartPosition(20);

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
                          CoralScorerSubsystem coralScorerSubsystem) {
        this.autonomousCommandSelector = autonomousCommandSelector;
        this.coralScorerSubsystem = coralScorerSubsystem;
    }

    public LightsStateMessage getCurrentState() {
        boolean dsEnabled = DriverStation.isEnabled();
        LightsStateMessage currentState;

        // Needs to implement vision as well
        // Not sure about if the way we are checking the shooter is correct (and collector)
        if (!dsEnabled) {
            currentState = LightsStateMessage.RobotDisabled;
        } else if (coralScorerSubsystem.confidentlyHasCoral()) {
            currentState = LightsStateMessage.CoralPresent;
        } else if (coralScorerSubsystem.getCoralScorerState() == CoralScorerSubsystem.CoralScorerState.INTAKING) {
            currentState = LightsStateMessage.CoralReadyToReceive;
        } else {
            currentState = LightsStateMessage.RobotEnabled;
        }
        return currentState;
    }

    public void sendState(LightsStateMessage state) {
        var bits = convertIntToBits(state.getValue());
        // TODO: decide on how communication will actually happen
    }

    public LightsStateMessage getState() {
        return state;
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
    public static boolean[] convertIntToBits(int value) {
        boolean[] bits = new boolean[numBits];
        for(int i = 0; i < numBits; i++) {
            bits[i] = (value & (1 << i)) != 0;
        }
        return bits;
    }

    @Override
    public void periodic() {
        this.state = getCurrentState();
        sendState(state);

        aKitLog.record("LightState", state.toString());
    }  
}