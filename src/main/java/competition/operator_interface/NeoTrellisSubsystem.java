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




/**
 * General neotrellis map of buttons:
 * xooooxxh 1-8
 * oxCxxoxh 9-16
 * oxRPxoxh 17-24
 * xooooxxh 25-32
 * o - branch button
 * x - unused
 * h - height button
 * C - clear queue
 * D - remove algae
 * P - process algae
 *
 * Rather than creating ~48+ ChordButtons, this system instead monitors for the any combination
 * of a "branch button" and an "action button" being pressed. Once detected, the appropriate
 * scoring activity is placed in the scoring queue.
 */
@Singleton
public class NeoTrellisSubsystem extends BaseSubsystem {

    final OperatorInterface oi;
    final XJoystick neoTrellis;
    final ScoringQueue scoringQueue;

    final HashMap<FaceBranch, Integer> locationsToButtonIndices;
    final HashMap<Landmarks.CoralLevel, Integer> levelsToButtonIndices;

    final HashMap<FaceBranch, AdvancedTrigger> locationsToButtons;
    final HashMap<Landmarks.CoralLevel, AdvancedTrigger> levelsToButtons;
    final AdvancedTrigger removeAlgaeButton;
    final AdvancedTrigger processAlgaeButton;
    final AdvancedTrigger resetQueueButton;

    final Latch comboDetectedLatch;

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

        // Resetting doesn't need any other button to be pressed, so we set it up as a typical
        // "press this button and get this command" binding.
        resetQueueButton.onTrue(new InstantCommand(scoringQueue::clearQueue).ignoringDisable(true));

        comboDetectedLatch = new Latch(
                false,
                Latch.EdgeType.RisingEdge,
                this::queueAppropriateAction);
    }

    protected void initializeLocationsAndLevels() {
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

    protected void initializeLevel(Landmarks.CoralLevel level, int buttonNumber) {
        levelsToButtons.put(level, neoTrellis.getifAvailable(buttonNumber));
        levelsToButtonIndices.put(level, buttonNumber);
    }

    protected void initializeLocationPair(Landmarks.ReefFace face, int buttonA, int buttonB) {
        initializeLocation(face, Landmarks.Branch.A, buttonA);
        initializeLocation(face, Landmarks.Branch.B, buttonB);
    }

    protected void initializeLocation(Landmarks.ReefFace face, Landmarks.Branch branch, int buttonIndex) {
        var faceBranch = new FaceBranch(face, branch);
        locationsToButtons.put(faceBranch, neoTrellis.getifAvailable(buttonIndex));
        locationsToButtonIndices.put(faceBranch, buttonIndex);
    }

    /**
     * This method is invoked when at least one location button and one action button are pressed.
     * It should only take action on the "rising edge" of such an event to debounce the buttons.
     * @param e EdgeType (from the Latch)
     */
    protected void queueAppropriateAction(Latch.EdgeType e) {
        if (e != Latch.EdgeType.RisingEdge) {
            // If this is a falling edge, we don't care. We only want to act on the rising edge.
            return;
        }

        // Rising edge detected, aka a legal combo press was detected. Find the first indicator
        // in each category (location, action) and queue it for the Oracle (or anything else that's
        // listening to the ScoringQueue).

        FaceBranch location = new FaceBranch(Landmarks.ReefFace.CLOSE, Landmarks.Branch.A);

        // Find the first location button that's currently active.
        for (var locationEntry : locationsToButtons.entrySet()) {
            if (locationEntry.getValue().getAsBoolean()) {
                location = locationEntry.getKey();
                break;
            }
        }

        // Find the first coral button that's currently active
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

        // Somehow, we are in a state where we have a location button pressed, but no action button,
        // but the latch is still high.
        log.warn("Combo press detected, but no action seems valid. No action will be added to the ScoringQueue.");
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


}