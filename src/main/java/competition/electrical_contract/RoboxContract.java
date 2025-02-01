package competition.electrical_contract;

import javax.inject.Inject;

public class RoboxContract extends Contract2025 {
    @Inject
    public RoboxContract() {}

    @Override
    public boolean isElevatorDistanceSensorReady() {
        return true;
    }
}
