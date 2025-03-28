package competition.aspects;

import edu.wpi.first.wpilibj.DriverStation;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import org.aspectj.lang.ProceedingJoinPoint;
import org.aspectj.lang.annotation.Around;
import org.aspectj.lang.annotation.Aspect;

@Aspect
public class DriverStationOverrides {
    public static final Logger log = LogManager.getLogger(DriverStation.class);

    @Around("execution(* edu.wpi.first.wpilibj.DriverStation.reportWarning(java.lang.String, boolean))")
    public void reportWarningOverride(ProceedingJoinPoint joinPoint) {
        log.warn(joinPoint.getArgs()[0]);
    }

    @Around("execution(* edu.wpi.first.wpilibj.Watchdog.printEpochs(..))")
    public void disableLoopOverrunPrinting(ProceedingJoinPoint joinPoint) {
        // Do nothing
    }
}
