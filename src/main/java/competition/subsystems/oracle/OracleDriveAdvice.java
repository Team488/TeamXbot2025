package competition.subsystems.oracle;

import xbot.common.math.XYPair;
import xbot.common.trajectory.XbotSwervePoint;

import java.util.List;

public record OracleDriveAdvice (
        int instructionNumber,
        OracleSubsystem.DriveAdviceMode mode,
        List<XbotSwervePoint> path,
        FieldOrientedDriveData fieldOrientedIntents) {}