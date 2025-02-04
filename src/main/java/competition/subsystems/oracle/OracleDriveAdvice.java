package competition.subsystems.oracle;

import xbot.common.trajectory.XbotSwervePoint;

import java.util.List;

public record OracleDriveAdvice (int instructionNumber, List<XbotSwervePoint> path) {}