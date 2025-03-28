package competition.operator_interface;

import java.util.HashMap;
import java.util.HashSet;

import javax.inject.Inject;

import competition.commandgroups.DriveToReefFaceThenAlignCommandGroupFactory;
import competition.subsystems.oracle.FaceBranch;
import competition.subsystems.oracle.ScoringQueue;
import competition.subsystems.pose.Landmarks;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import xbot.common.command.BaseSubsystem;
import xbot.common.controls.sensors.XJoystick;
import xbot.common.logic.Latch;

public class OIReefSubsystem extends BaseSubsystem {
    final OperatorInterface oi;
    final XJoystick oiReef;
    final ScoringQueue scoringQueue;

    final DriveToReefFaceThenAlignCommandGroupFactory driveToReefFaceThenAlignCommandGroupFactory;

    final HashMap<FaceBranch, Integer> buttonMap;
    final HashSet<Latch> latchSet;

    @Inject
    public OIReefSubsystem(OperatorInterface oi,
            ScoringQueue scoringQueue,
            DriveToReefFaceThenAlignCommandGroupFactory driveToReefFaceThenAlignCommandGroupFactory) {
        this.oi = oi;
        this.oiReef = oi.oiReef;
        this.scoringQueue = scoringQueue;

        this.driveToReefFaceThenAlignCommandGroupFactory = driveToReefFaceThenAlignCommandGroupFactory;

        this.buttonMap = new HashMap<>();
        this.latchSet = new HashSet<>();

        initializeButtons();
        setUpButtonCommands();
    }

    protected void initializeButtons() {
        buttonMap.put(new FaceBranch(Landmarks.ReefFace.CLOSE, Landmarks.Branch.A), 1);
        buttonMap.put(new FaceBranch(Landmarks.ReefFace.CLOSE, Landmarks.Branch.B), 2);
        buttonMap.put(new FaceBranch(Landmarks.ReefFace.CLOSE_RIGHT, Landmarks.Branch.A), 3);
        buttonMap.put(new FaceBranch(Landmarks.ReefFace.CLOSE_RIGHT, Landmarks.Branch.B), 4);
        buttonMap.put(new FaceBranch(Landmarks.ReefFace.FAR_RIGHT, Landmarks.Branch.A), 5);
        buttonMap.put(new FaceBranch(Landmarks.ReefFace.FAR_RIGHT, Landmarks.Branch.B), 6);
        buttonMap.put(new FaceBranch(Landmarks.ReefFace.FAR, Landmarks.Branch.A), 7);
        buttonMap.put(new FaceBranch(Landmarks.ReefFace.FAR, Landmarks.Branch.B), 8);
        buttonMap.put(new FaceBranch(Landmarks.ReefFace.FAR_LEFT, Landmarks.Branch.A), 9);
        buttonMap.put(new FaceBranch(Landmarks.ReefFace.FAR_LEFT, Landmarks.Branch.B), 10);
        buttonMap.put(new FaceBranch(Landmarks.ReefFace.CLOSE_LEFT, Landmarks.Branch.A), 11);
        buttonMap.put(new FaceBranch(Landmarks.ReefFace.CLOSE_LEFT, Landmarks.Branch.B), 12);
    }

    public void setUpButtonCommands() {
        for (FaceBranch faceBranch : buttonMap.keySet()) {
            int channel = buttonMap.get(faceBranch);
            SequentialCommandGroup driveToReefCommand = driveToReefFaceThenAlignCommandGroupFactory.create(faceBranch.face(), faceBranch.branch());
            oiReef.getifAvailable(channel).onTrue(driveToReefCommand);
        }
    }



    @Override
    public void periodic() {
        
    }
}
