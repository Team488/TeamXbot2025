package competition.subsystems.pose;

public enum Cameras {
    FRONT_LEFT_CAMERA(0),
    FRONT_RIGHT_CAMERA(1),
    BACK_CAMERA(2);

    private final int index;

    Cameras(int index) {
        this.index = index;
    }

    public int getIndex() {
        return index;
    }
}
