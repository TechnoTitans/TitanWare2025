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
    private final static Set<Command> running = new HashSet<>();
    private final static Map<Command, Command> interrupters = new HashMap<>();

    private LoggedCommandScheduler() {
    }

    public static void init(final CommandScheduler commandScheduler) {
        commandScheduler.onCommandInitialize(command -> {
            if (!interrupters.containsKey(command)) {
                running.add(command);
            }
        });
        commandScheduler.onCommandFinish(command -> {
            running.remove(command);
            interrupters.remove(command);
        });

        commandScheduler.onCommandInterrupt((interrupted, interrupting) -> {
            running.remove(interrupted);
            interrupters.remove(interrupted);
            interrupting.ifPresent(interrupter -> interrupters.put(interrupter, interrupted));

//            Logger.recordOutput(
//                    "Commands/InterruptedRequirements",
//                    LogUtils.getRequirementsFromSubsystems(interrupted.getRequirements())
//            );
//            Logger.recordOutput(
//                    "Commands/Interrupter",
//                    interrupting.isPresent() ? interrupting.get().getName() : "None"
//            );
//            Logger.recordOutput(
//                    "Commands/InterrupterRequirements",
//                    LogUtils.getRequirementsFromSubsystems(
//                            interrupting.isPresent() ? interrupting.get().getRequirements() : Set.of()
//                    )
//            );
        });
    }

    public static void periodic() {
        Logger.recordOutput("Commands/Running/.type", "Alerts");

        final String[] running = new String[LoggedCommandScheduler.running.size()];
        {
            int i = 0;
            for (final Command command : LoggedCommandScheduler.running) {
                running[i] = command.getName();
                i++;
            }
        }
        Logger.recordOutput("Commands/Running/warnings", running);

        final String[] interrupters = new String[LoggedCommandScheduler.interrupters.size()];
        {
            int i = 0;
            for (final Map.Entry<Command, Command> entry : LoggedCommandScheduler.interrupters.entrySet()) {
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
        Logger.recordOutput("Commands/Running/errors", interrupters);
    }
}