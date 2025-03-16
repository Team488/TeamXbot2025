package competition.aspects;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import org.aspectj.lang.JoinPoint;
import org.aspectj.lang.annotation.Aspect;
import org.aspectj.lang.annotation.Before;
import xbot.common.command.BaseMaintainerCommand;

import java.util.HashMap;
import java.util.Map;

// CHECKSTYLE:OFF
@Aspect
public class CommandTracer {

    private final Map<Command, Alert> runningCommandAlerts = new HashMap<>();

    @Before("execution(* edu.wpi.first.wpilibj2.command.Command+.initialize(..))" +
            "|| execution(* edu.wpi.first.wpilibj2.command.Command+.execute(..))")
    public void createCommandAlert(JoinPoint joinPoint) {
        if (!runningCommandAlerts.containsKey((Command)joinPoint.getThis())) {
            var command = (Command)joinPoint.getThis();
            var alertType = command instanceof BaseMaintainerCommand<?> ? Alert.AlertType.kWarning : Alert.AlertType.kInfo;
            var alert = new Alert("RunningCommands", command.getName(), alertType);
            alert.set(true);
            runningCommandAlerts.put(command, alert);
        }
    }

    @Before("execution(* edu.wpi.first.wpilibj2.command.Command+.end(..))")
    public void clearCommandAlert(JoinPoint joinPoint) {
        if (runningCommandAlerts.containsKey((Command)joinPoint.getThis())) {
            runningCommandAlerts.remove((Command)joinPoint.getThis()).close();
        }
    }
}
// CHECKSTYLE:ON
