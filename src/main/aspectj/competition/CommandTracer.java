package competition;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Timer;
import org.aspectj.lang.JoinPoint;
import org.aspectj.lang.annotation.Aspect;
import org.aspectj.lang.annotation.Before;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.Logger;
import xbot.common.command.BaseRobot;
import xbot.common.controls.sensors.XTimer;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.logging.FileHandler;

@Aspect
public class CommandTracer {
    BufferedWriter writer;
    boolean openedFile;

    public static final String PREFERENCES_KEY = "TraceFunctionCalls";

    public CommandTracer() {
        try {
            String basePath;
            if (BaseRobot.isSimulation()) {
                basePath = "";
            } else {
                basePath = "/U/logs/";
            }
            this.writer = new BufferedWriter(new FileWriter(
                    new File(basePath + "CommandTracer.log")
            ));
            this.openedFile = true;
        } catch (IOException e) {
            this.writer = null;
            this.openedFile = false;
        }
    }

    @Before("execution(* edu.wpi.first.wpilibj.IterativeRobotBase.loopFunc(..))")
    public void logRobotLoop(JoinPoint joinPoint) {
        var enableTracing = Preferences.getBoolean(PREFERENCES_KEY, false);
        if (openedFile && enableTracing) {
            try {
                writer.write(String.format("%f: %s\n", Timer.getTimestamp(), joinPoint.toShortString()));
            } catch (IOException ignored) {}
        }
    }

    @Before("execution(* edu.wpi.first.wpilibj2.command.Command+.initialize(..))" +
            "|| execution(* edu.wpi.first.wpilibj2.command.Command+.execute(..))" +
            "|| execution(* edu.wpi.first.wpilibj2.command.Command+.end(..))")
    public void logCommand(JoinPoint joinPoint) {
        var enableTracing = Preferences.getBoolean(PREFERENCES_KEY, false);
        if (openedFile && enableTracing) {
            try {
                writer.write(String.format("%f: -> %s\n", Timer.getTimestamp(), joinPoint.toShortString()));
            } catch (IOException ignored) {}
        }
    }
}
