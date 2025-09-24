package frc.robot.utils.logging;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.constants.Constants;
import frc.robot.utils.commands.LoggedTrigger;
import org.littletonrobotics.junction.Logger;

import java.util.*;
import java.util.function.DoubleSupplier;

public class LoggedCommandScheduler {
    private static class DelayedBuffer<T> implements Iterable<T> {
        private final DoubleSupplier timeSource;
        private final NavigableMap<Double, T> input = new TreeMap<>();
        private final List<T> buffer = new ArrayList<>();

        public DelayedBuffer(final DoubleSupplier timeSource) {
            this.timeSource = timeSource;
        }

        /**
         * Add an item to the buffer.
         *
         * @param item The item to add.
         * @param delaySeconds The delay before adding the item.
         */
        public void add(final T item, final double delaySeconds) {
            final double now = timeSource.getAsDouble();
            input.put(now + delaySeconds, item);
            process(now);
        }

        public int size() {
            return buffer.size();
        }

        @Override
        public Iterator<T> iterator() {
            process(timeSource.getAsDouble());
            return buffer.iterator();
        }

        private void process(final double now) {
            while (!input.isEmpty()) {
                final Map.Entry<Double, T> entry = input.firstEntry();
                final double delayTill = entry.getKey();
                if (delayTill >= now) {
                    buffer.add(entry.getValue());
                } else {
                    return;
                }
            }
        }
    }

    private static final String LogKey = "Commands";
    private static final String AlertType = "Alerts";

    private static final Set<Command> runningNonInterrupters = new HashSet<>();
    private static final Map<Command, Command> runningInterrupters = new HashMap<>();
    private static final Map<Subsystem, Command> requiredSubsystems = new HashMap<>();

    private static final DelayedBuffer<Command> headerTimestamps =
            new DelayedBuffer<>(Timer::getTimestamp);

    private static final Map<Command, LoggedTrigger> scheduledBy = new HashMap<>();
    private static final Map<Command, LoggedTrigger> cancelledCausedBy = new HashMap<>();

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

    public static void scheduledBy(final Command scheduled, final LoggedTrigger by) {
        scheduledBy.put(scheduled, by);
    }

    private static void logRunningCommands() {
        Logger.recordOutput(LogKey + "/Running/.type", AlertType);

        final Set<Command> runningNonInterrupters = LoggedCommandScheduler.runningNonInterrupters;
        final String[] running = new String[runningNonInterrupters.size()];
        {
            int i = 0;
            for (final Command command : runningNonInterrupters) {
                running[i] = command.getName();
                if (scheduledBy.containsKey(command)) {
                    headerTimestamps.add(
                            command,
                            12 * Constants.LOOP_PERIOD_SECONDS
                    );
                }
                i++;
            }
        }
        Logger.recordOutput(LogKey + "/Running/warnings", running);

        final int nAnnotations = 2;
        final String[] annotations = new String[nAnnotations * headerTimestamps.size()];
        {
            int i = 0;
            for (final Iterator<Command> it = headerTimestamps.iterator(); it.hasNext(); ) {
                final Command command = it.next();
                final LoggedTrigger trigger = scheduledBy.get(command);
                scheduledBy.remove(command);

                annotations[i] = "scheduled by: " + trigger.getName();
                annotations[i + 1] = String.valueOf(trigger.getAsBoolean());

                i += nAnnotations;
                it.remove();
            }
        }
        Logger.recordOutput(LogKey + "/Running/infos", annotations);

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