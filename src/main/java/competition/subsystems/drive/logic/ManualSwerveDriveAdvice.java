package competition.subsystems.drive.logic;

import xbot.common.math.XYPair;

public record ManualSwerveDriveAdvice(XYPair translation, double rotationIntent, double currentHeading, XYPair centerOfRotationInches) {
}
