package competition.electrical_contract;

import javax.inject.Inject;

public class UnitTestContract2025 extends Contract2025 {

    @Inject
    public UnitTestContract2025() {}

    @Override
    public boolean isDriveReady(){
        return true;
    }

    @Override
    public boolean isAlgaeCollectionReady(){
        return true;
    }

    @Override
    public boolean isCoralCollectionMotorReady(){
        return true;
    }

    @Override
    public boolean isElevatorReady(){
        return true;
    }
}
