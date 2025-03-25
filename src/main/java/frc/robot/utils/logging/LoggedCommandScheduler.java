package frc.robot.utils.logging;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.littletonrobotics.junction.Logger;

import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

public class LoggedCommandScheduler {
    private static final String LogKey = "Commands";
    private static final String AlertType = "Alerts";

    private static final Set<Command> runningNonInterrupters = new HashSet<>();
    private static final Map<Command, Command> runningInterrupters = new HashMap<>();
    private static final Map<Subsystem, Command> requiredSubsystems = new HashMap<>();

    private LoggedCommandScheduler() {
    }

    private static void commandStarted(final Command command) {
        if (!runningInterrupters.containsKey(command)) {
            runningNonInterrupters.add(command);
        }

        for (final Subsystem subsystem : command.getRequirements()) {
            requiredSubsystems.put(subsystem, command);
        }
    }

    private static void commandEnded(final Command command) {
        runningNonInterrupters.remove(command);
        runningInterrupters.remove(command);

        for (final Subsystem subsystem : command.getRequirements()) {
            requiredSubsystems.remove(subsystem);
        }
    }

    public static void init(final CommandScheduler commandScheduler) {
        commandScheduler.onCommandInitialize(LoggedCommandScheduler::commandStarted);
        commandScheduler.onCommandFinish(LoggedCommandScheduler::commandEnded);

        commandScheduler.onCommandInterrupt((interrupted, interrupting) -> {
            interrupting.ifPresent(interrupter -> runningInterrupters.put(interrupter, interrupted));
            commandEnded(interrupted);
        });
    }

    private static void logRunningCommands() {
        Logger.recordOutput(LogKey + "/Running/.type", AlertType);

        final Set<Command> runningNonInterrupters = LoggedCommandScheduler.runningNonInterrupters;
        final String[] running = new String[runningNonInterrupters.size()];
        {
            int i = 0;
            for (final Command command : runningNonInterrupters) {
                running[i] = command.getName();
                i++;
            }
        }
        Logger.recordOutput(LogKey + "/Running/warnings", running);

        final Map<Command, Command> runningInterrupters = LoggedCommandScheduler.runningInterrupters;
        final String[] interrupters = new String[runningInterrupters.size()];
        {
            int i = 0;
            for (final Map.Entry<Command, Command> entry : runningInterrupters.entrySet()) {
                final Command interrupter = entry.getKey();
                final Command interrupted = entry.getValue();

                final Set<Subsystem> commonRequirements = new HashSet<>(interrupter.getRequirements());
                commonRequirements.retainAll(interrupted.getRequirements());

                final StringBuilder requirements = new StringBuilder();
                int j = 1;
                for (final Subsystem subsystem : commonRequirements) {
                    requirements.append(subsystem.getName());
                    if (j < commonRequirements.size()) {
                        requirements.append(",");
                    }

                    j++;
                }

                interrupters[i] = interrupter.getName()
                        + " interrupted "
                        + interrupted.getName()
                        + " (" + requirements + ")";
                i++;
            }
        }
        Logger.recordOutput(LogKey + "/Running/errors", interrupters);
    }

    private static void logRequiredSubsystems() {
        Logger.recordOutput(LogKey + "/Subsystems/.type", AlertType);

        final Map<Subsystem, Command> requiredSubsystems = LoggedCommandScheduler.requiredSubsystems;
        final String[] subsystems = new String[requiredSubsystems.size()];
        {
            int i = 0;
            for (final Map.Entry<Subsystem, Command> entry : requiredSubsystems.entrySet()) {
                final Subsystem required = entry.getKey();
                final Command command = entry.getValue();

                subsystems[i] = required.getName()
                        + " (" + command.getName() + ")";
                i++;
            }
        }
        Logger.recordOutput(LogKey + "/Subsystems/infos", subsystems);
    }

    public static void periodic() {
        logRunningCommands();
        logRequiredSubsystems();
    }
}