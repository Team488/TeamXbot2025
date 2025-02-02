package competition.subsystems.oracle;

import competition.subsystems.coral_scorer.CoralScorerSubsystem;
import competition.subsystems.pose.Landmarks;
import xbot.common.trajectory.XbotSwervePoint;

import java.util.List;

public class OracleSuperstructureAdvice {
    public int instructionNumber;
    public Landmarks.CoralLevel coralLevelToAchieve;
    public CoralScorerSubsystem.CoralScorerState desiredScorerState;
}
