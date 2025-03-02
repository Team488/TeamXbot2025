package competition.subsystems.SysID;

import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import xbot.common.command.BaseSubsystem;

public class SysIDSubsystem extends BaseSubsystem {
//    private final SysIdRoutine routine;
    public SysIDSubsystem() {
        SysIdRoutine.Config config = new SysIdRoutine.Config();
//        routine = new SysIdRoutine();
    }
}
