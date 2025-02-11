package competition.operator_interface;

import competition.subsystems.oracle.FaceBranch;
import competition.subsystems.oracle.ScoringQueue;
import competition.subsystems.pose.Landmarks;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import xbot.common.command.BaseSubsystem;
import xbot.common.controls.sensors.XJoystick;
import xbot.common.controls.sensors.buttons.AdvancedTrigger;
import xbot.common.logic.Latch;

import javax.inject.Inject;
import javax.inject.Singleton;
import java.util.HashMap;

@Singleton
public class NeoTrellisSubsystem extends BaseSubsystem {

    final OperatorInterface oi;
    final XJoystick neoTrellis;
    final ScoringQueue scoringQueue;

    HashMap<FaceBranch, Integer> locationsToButtonIndices;
    HashMap<Landmarks.CoralLevel, Integer> levelsToButtonIndices;

    HashMap<FaceBranch, AdvancedTrigger> locationsToButtons;
    HashMap<Landmarks.CoralLevel, AdvancedTrigger> levelsToButtons;
    final AdvancedTrigger removeAlgaeButton;
    final AdvancedTrigger processAlgaeButton;
    final AdvancedTrigger resetQueueButton;

    final Latch comboDetectedLatch;

    public enum NeoTrellisButtonNames {
        NearA(1),
        NearB(2);

        int buttonIndex = 0;
        NeoTrellisButtonNames(int buttonIndex) {
            this.buttonIndex = this.buttonIndex;
        }

        public int getButtonIndex() {
            return buttonIndex;
        }
    }

    @Inject
    public NeoTrellisSubsystem(OperatorInterface oi, ScoringQueue scoringQueue) {
        this.oi = oi;
        this.neoTrellis = oi.neoTrellis;
        this.scoringQueue = scoringQueue;

        locationsToButtons = new HashMap<>();
        levelsToButtons = new HashMap<>();

        locationsToButtonIndices = new HashMap<>();
        levelsToButtonIndices = new HashMap<>();

        initializeLocationsAndLevels();
        removeAlgaeButton = neoTrellis.getifAvailable(19);
        processAlgaeButton = neoTrellis.getifAvailable(20);
        resetQueueButton = neoTrellis.getifAvailable(11);

        resetQueueButton.onTrue(new InstantCommand(scoringQueue::clearQueue));

        comboDetectedLatch = new Latch(
                false,
                Latch.EdgeType.RisingEdge,
                this::queueAppropriateAction);
    }

    private void initializeLocationsAndLevels() {
        initializeLocationPair(Landmarks.ReefFace.CLOSE, 27, 28);
        initializeLocationPair(Landmarks.ReefFace.CLOSE_RIGHT, 29, 22);
        initializeLocationPair(Landmarks.ReefFace.FAR_RIGHT, 14, 5);
        initializeLocationPair(Landmarks.ReefFace.FAR, 4, 3);
        initializeLocationPair(Landmarks.ReefFace.FAR_LEFT, 2, 9);
        initializeLocationPair(Landmarks.ReefFace.CLOSE_LEFT,17, 26);

        initializeLevel(Landmarks.CoralLevel.ONE, 32);
        initializeLevel(Landmarks.CoralLevel.TWO, 24);
        initializeLevel(Landmarks.CoralLevel.THREE, 16);
        initializeLevel(Landmarks.CoralLevel.FOUR, 8);
    }

    private void initializeLevel(Landmarks.CoralLevel level, int buttonNumber) {
        levelsToButtons.put(level, neoTrellis.getifAvailable(buttonNumber));
        levelsToButtonIndices.put(level, buttonNumber);
    }

    private void initializeLocationPair(Landmarks.ReefFace face, int buttonA, int buttonB) {
        initializeLocation(face, Landmarks.Branch.A, buttonA);
        initializeLocation(face, Landmarks.Branch.B, buttonB);
    }

    private void initializeLocation(Landmarks.ReefFace face, Landmarks.Branch branch, int buttonIndex) {
        var faceBranch = new FaceBranch(face, branch);
        locationsToButtons.put(faceBranch, neoTrellis.getifAvailable(buttonIndex));
        locationsToButtonIndices.put(faceBranch, buttonIndex);
    }

    private void queueAppropriateAction(Latch.EdgeType e) {
        if (e != Latch.EdgeType.RisingEdge) {
            return;
        }

        // Rising edge detected, aka a legal combo press was detected. Find the first indicator
        // in each category (location, action) and queue it for the Oracle.

        FaceBranch location = new FaceBranch(Landmarks.ReefFace.CLOSE, Landmarks.Branch.A);

        for (var locationEntry : locationsToButtons.entrySet()) {
            if (locationEntry.getValue().getAsBoolean()) {
                location = locationEntry.getKey();
                break;
            }
        }

        // Now check possible actions
        for (var heightEntry : levelsToButtons.entrySet()) {
            if (heightEntry.getValue().getAsBoolean()) {
                scoringQueue.addCoralTask(
                        location.face(),
                        location.branch(),
                        heightEntry.getKey()
                );
                return;
            }
        }

        // If we didn't get a coral task, check algae
        if (removeAlgaeButton.getAsBoolean()) {
            scoringQueue.addAlgaeRemovalTask(location.face());
            return;
        }

        // If we didn't get any other task, try processing
        if (processAlgaeButton.getAsBoolean()) {
            scoringQueue.addAlgaeProcessingTask();
            return;
        }
    }

    @Override
    public void periodic() {
        // is any location button pressed
        boolean anyLocationPressed = false;
        for (AdvancedTrigger button : locationsToButtons.values()) {
            if (button.getAsBoolean()) {
                anyLocationPressed = true;
                break;
            }
        }

        // is any height pressed
        boolean anyHeightPressed = false;
        for (AdvancedTrigger button : levelsToButtons.values()) {
            if (button.getAsBoolean()) {
                anyHeightPressed = true;
                break;
            }
        }

        boolean isAnyActionPressed =
                anyHeightPressed || processAlgaeButton.getAsBoolean() || removeAlgaeButton.getAsBoolean();

        comboDetectedLatch.setValue(anyLocationPressed && isAnyActionPressed);
    }

    public int getNeoTrellisButtonIndex(Landmarks.ReefFace face, Landmarks.Branch branch) {
        return locationsToButtonIndices.get(new FaceBranch(face, branch));
    }

    public int getNeoTrellisButtonIndex(Landmarks.CoralLevel level) {
        return levelsToButtonIndices.get(level);
    }

    public int getNeoTrellisButtonAlgaeRemoval() {
        return 19;
    }

    public int getNeoTrellisButtonAlgaeProcessing() {
        return 20;
    }

    public int getNeoTrellisButtonClearQueue() {
        return 11;
    }

    // General neotrellis idea
        // xooooxxh 1-8
        // oxCxxoxh 9-16
        // oxRPxoxh 17-24
        // xooooxxh 25-32
        // o - branch button
        // x - unused
        // h - height button
        // C - clear queue
        // D - remove algae
        // P - process algae

        // This would normally create a gajillion buttons, which seems gross. Maybe a subsystem that just
        // checks in periodic? But then I have to do all the debouncing myself.

        // could check for the rising edge of a button from each "category" being true. When that's the case,
        // evaluate the buttons and take the appropriate action
}