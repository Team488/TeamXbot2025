package competition.aspects;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import org.aspectj.lang.JoinPoint;
import org.aspectj.lang.annotation.Aspect;
import org.aspectj.lang.annotation.Before;
import xbot.common.command.BaseRobot;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

// CHECKSTYLE:OFF
@Aspect
public class CommandTracer {
    BufferedWriter writer;
    boolean openedFile;

    public static final String PREFERENCES_KEY = "TraceFunctionCalls";

    public CommandTracer() {
        if (!Preferences.containsKey(CommandTracer.PREFERENCES_KEY)) {
            Preferences.setBoolean(CommandTracer.PREFERENCES_KEY, false);
        }

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
                writer.write(String.format("%f: -> %s %s\n", Timer.getTimestamp(), ((Command)joinPoint.getThis()).getName(), joinPoint.toShortString()));
            } catch (IOException ignored) {}
        }
    }
}
// CHECKSTYLE:ON
