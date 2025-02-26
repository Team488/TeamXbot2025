package competition.electrical_contract;

import javax.inject.Inject;

public class UnitTestContract2025 extends Contract2025 {

    @Inject
    public UnitTestContract2025() {}



    public boolean isAlgaeCollectionReady() { return true; }

    public boolean isCoralCollectionMotorReady() { return true; }

    public boolean isCoralScorerSensorReady() { return true;}

    public boolean isCoralArmMotorReady() { return true; }

    public boolean isCoralArmPivotAbsoluteEncoderReady() { return true; }

    public boolean isCoralArmLowSensorReady() { return true; }

    public boolean isElevatorReady() { return true;}

    public boolean isDriveReady() { return true;}

    public boolean areCanCodersReady() { return true; }

    @Override
    public boolean isHumanLoadRampReady() { return true; }

    public boolean isElevatorBottomSensorReady() { return true; }

    public boolean isAlgaeArmBottomSensorReady() { return true; }

    public boolean isAlgaeArmPivotMotorReady() { return true; }

    public boolean isElevatorDistanceSensorReady() { return true ; }

}
