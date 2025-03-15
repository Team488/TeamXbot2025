package competition.aspects;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import org.aspectj.lang.JoinPoint;
import org.aspectj.lang.ProceedingJoinPoint;
import org.aspectj.lang.annotation.After;
import org.aspectj.lang.annotation.Around;
import org.aspectj.lang.annotation.Aspect;
import org.aspectj.lang.annotation.Before;
import org.littletonrobotics.junction.Logger;
import xbot.common.command.BaseSubsystem;

import java.io.IOException;
import java.util.HashMap;
import java.util.Map;

@Aspect
public class SubsystemTracer {

    public Map<BaseSubsystem, Long> refreshDataFrameTimes = new HashMap<>();
    public Map<BaseSubsystem, Long> periodicTimes = new HashMap<>();

    @Before("execution(* edu.wpi.first.wpilibj.IterativeRobotBase.loopFunc(..))")
    public void clearMeasurements(JoinPoint joinPoint) {
        refreshDataFrameTimes.clear();
        periodicTimes.clear();
    }

    @After("execution(* edu.wpi.first.wpilibj.IterativeRobotBase.loopFunc(..))")
    public void reportMeasurements(JoinPoint joinPoint) {
        // both should be the same length
        if (refreshDataFrameTimes.isEmpty()) {
            return;
        }

        Logger.recordOutput("Performance/Subsystems/Total/refreshDataFrame",
                refreshDataFrameTimes.values().stream().reduce(0L, Long::sum));
        Logger.recordOutput("Performance/Subsystems/Total/periodic",
                periodicTimes.values().stream().reduce(0L, Long::sum));
        Logger.recordOutput("Performance/Subsystems/Total/total",
                refreshDataFrameTimes.values().stream().reduce(0L, Long::sum)
                + periodicTimes.values().stream().reduce(0L, Long::sum));
    }

    @Around("execution(* xbot.common.command.BaseSubsystem+.refreshDataFrame())")
    public void measureSubsystemRefreshDataFrame(ProceedingJoinPoint joinPoint) throws Throwable {
        var startTime = RobotController.getFPGATime();
        joinPoint.proceed();
        var endTime = RobotController.getFPGATime();

        var subsystem = (BaseSubsystem)joinPoint.getThis();
        var time = endTime - startTime;
        refreshDataFrameTimes.put(subsystem, time);

        Logger.runEveryN(5, () -> Logger.recordOutput("Performance/Subsystems/"
                + subsystem.getName() + subsystem.hashCode()
                + "/refreshDataFrame", time));
    }

    @Around("execution(* xbot.common.command.BaseSubsystem+.periodic())")
    public void measureSubsystemPeriodic(ProceedingJoinPoint joinPoint) throws Throwable {
        var startTime = RobotController.getFPGATime();
        joinPoint.proceed();
        var endTime = RobotController.getFPGATime();

        var subsystem = (BaseSubsystem)joinPoint.getThis();
        var time = endTime - startTime;
        periodicTimes.put(subsystem, time);

        Logger.runEveryN(5, () -> Logger.recordOutput("Performance/Subsystems/"
                + subsystem.getName() + subsystem.hashCode()
                + "/periodic", time));
    }
}
