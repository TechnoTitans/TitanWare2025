package frc.robot.utils.commands;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import java.util.ArrayList;
import java.util.List;

/**
 * A command composition that runs a list of commands in sequence. But faster.
 * Generously donated by 254.
 * See <a href="https://github.com/Team254/FRC-2025-Public/blob/main/src/main/java/com/team254/lib/commands/ChezySequenceCommandGroup.java">...</a>
 *
 * <p>The rules for command compositions apply: command instances that are passed to it cannot be
 * added to any other composition or scheduled individually, and the composition requires all
 * subsystems its components require.
 */
public class FastSequentialCommandGroup extends Command {
    private final List<Command> commands = new ArrayList<>();
    private int currentCommandIndex = -1;
    private boolean runWhenDisabled = true;
    private InterruptionBehavior interruptBehavior = InterruptionBehavior.kCancelIncoming;

    /**
     * Creates a new SequentialCommandGroup. The given commands will be run sequentially, with the
     * composition finishing when the last command finishes.
     *
     * @param commands the commands to include in this composition.
     */
    public FastSequentialCommandGroup(final Command... commands) {
        addCommands(commands);
    }

    /**
     * Adds the given commands to the group.
     *
     * @param commands Commands to add, in order of execution.
     */
    public final void addCommands(final Command... commands) {
        if (currentCommandIndex != -1) {
            throw new IllegalStateException(
                    "Commands cannot be added to a composition while it's running");
        }

        CommandScheduler.getInstance().registerComposedCommands(commands);

        for (final Command command : commands) {
            this.commands.add(command);
            addRequirements(command.getRequirements());
            runWhenDisabled &= command.runsWhenDisabled();
            if (command.getInterruptionBehavior() == InterruptionBehavior.kCancelSelf) {
                interruptBehavior = InterruptionBehavior.kCancelSelf;
            }
        }
    }

    @Override
    public final void initialize() {
        currentCommandIndex = 0;

        if (!commands.isEmpty()) {
            commands.get(0).initialize();
        }
    }

    @Override
    public final void execute() {
        if (commands.isEmpty()) {
            return;
        }

        final Command currentCommand = commands.get(currentCommandIndex);
        currentCommand.execute();

        if (currentCommand.isFinished()) {
            currentCommand.end(false);
            currentCommandIndex++;
            if (currentCommandIndex < commands.size()) {
                commands.get(currentCommandIndex).initialize();

                // Go again to run the next loop.
                execute();
            }
        }
    }

    @Override
    public final void end(final boolean interrupted) {
        if (interrupted
                && !commands.isEmpty()
                && currentCommandIndex > -1
                && currentCommandIndex < commands.size()) {
            commands.get(currentCommandIndex).end(true);
        }
        currentCommandIndex = -1;
    }

    @Override
    public final boolean isFinished() {
        return currentCommandIndex == commands.size();
    }

    @Override
    public boolean runsWhenDisabled() {
        return runWhenDisabled;
    }

    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        return interruptBehavior;
    }

    @Override
    public void initSendable(final SendableBuilder builder) {
        super.initSendable(builder);

        builder.addIntegerProperty("index", () -> currentCommandIndex, null);
    }
}
