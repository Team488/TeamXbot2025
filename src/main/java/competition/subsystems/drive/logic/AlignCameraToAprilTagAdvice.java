package competition.subsystems.drive.logic;

import xbot.common.math.XYPair;

public record AlignCameraToAprilTagAdvice(
        XYPair driveIntent,
        double rotationIntent,
        AlignCameraToAprilTagCalculator.TagAcquisitionState tagAcquisitionState,
        AlignCameraToAprilTagCalculator.Activity activity){
}
