package frc.robot.utils.logging;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.utils.commands.LoggedTrigger;
import org.littletonrobotics.junction.Logger;

import java.util.*;


public class LoggedCommandScheduler {
    private static final String LogKey = "Commands";
    private static final String AlertType = "Alerts";

    private static final Set<Command> RunningNonInterrupters = new HashSet<>();
    private static final Map<Command, Command> RunningInterrupters = new HashMap<>();
    private static final Map<Subsystem, Command> RequiredSubsystems = new HashMap<>();

    private static final Map<Command, LoggedTrigger> ScheduledBy = new HashMap<>();
    private static final Set<Command> ScheduledBuffer = new LinkedHashSet<>();
    private static final String PadFirstTrigger = " ".repeat(4);
    private static final String PadRest = " ".repeat(8);

    private LoggedCommandScheduler() {
    }

    private static void commandStarted(final Command command) {
        if (!RunningInterrupters.containsKey(command)) {
            RunningNonInterrupters.add(command);
        }

        for (final Subsystem subsystem : command.getRequirements()) {
            RequiredSubsystems.put(subsystem, command);
        }
    }

    private static void commandEnded(final Command command) {
        RunningNonInterrupters.remove(command);
        RunningInterrupters.remove(command);
        ScheduledBy.remove(command);

        for (final Subsystem subsystem : command.getRequirements()) {
            RequiredSubsystems.remove(subsystem);
        }
    }

    public static void init(final CommandScheduler commandScheduler) {
        commandScheduler.onCommandInitialize(LoggedCommandScheduler::commandStarted);
        commandScheduler.onCommandFinish(LoggedCommandScheduler::commandEnded);

        commandScheduler.onCommandInterrupt((interrupted, interrupting) -> {
            interrupting.ifPresent(interrupter -> RunningInterrupters.put(interrupter, interrupted));
            commandEnded(interrupted);
        });
    }

    public static void scheduledBy(final Command scheduled, final LoggedTrigger by) {
        ScheduledBy.put(scheduled, by);
    }

    private static void logRunningCommands() {
        Logger.recordOutput(LogKey + "/Running/.type", AlertType);

        final Set<Command> runningNonInterrupters = LoggedCommandScheduler.RunningNonInterrupters;
        final String[] running = new String[runningNonInterrupters.size()];
        {
            int i = 0;
            for (final Command command : runningNonInterrupters) {
                running[i] = command.getName();
                if (ScheduledBy.containsKey(command)) {
                    ScheduledBuffer.add(command);
                }

                i++;
            }
        }

        final Map<Command, Command> runningInterrupters = LoggedCommandScheduler.RunningInterrupters;
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

                if (ScheduledBy.containsKey(interrupter)) {
                    ScheduledBuffer.add(interrupter);
                }

                i++;
            }
        }

        final String[] annotations;
        {
            final List<String> list = new ArrayList<>();
            for (final Iterator<Command> it = ScheduledBuffer.iterator(); it.hasNext(); ) {
                final Command command = it.next();
                if (!ScheduledBy.containsKey(command)) {
                    it.remove();
                    continue;
                }

                final LoggedTrigger trigger = Objects.requireNonNull(
                        ScheduledBy.get(command), "Missing ScheduledBy trigger");
                final String[] descriptor = trigger.getDescriptor();
                for (int j = descriptor.length - 1; j >= 0; j--) {
                    list.add((j == 0 ? PadFirstTrigger : PadRest) + descriptor[j]);
                }

                list.add(command.getName() + " scheduled by [" + trigger.id + "]:");
            }

            annotations = list.isEmpty() ? new String[0] : list.toArray(String[]::new);
        }

        Logger.recordOutput(LogKey + "/Running/infos", annotations);
        Logger.recordOutput(LogKey + "/Running/warnings", running);
        Logger.recordOutput(LogKey + "/Running/errors", interrupters);
    }

    private static void logRequiredSubsystems() {
        Logger.recordOutput(LogKey + "/Subsystems/.type", AlertType);

        final Map<Subsystem, Command> requiredSubsystems = LoggedCommandScheduler.RequiredSubsystems;
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