package competition.subsystems.oracle;

import competition.subsystems.coral_scorer.CoralScorerSubsystem;
import competition.subsystems.pose.Landmarks;

public record OracleSuperstructureAdvice (
        int instructionNumber,
        Landmarks.CoralLevel coralLevelToAchieve,
        CoralScorerSubsystem.CoralScorerState desiredScorerState) {}
