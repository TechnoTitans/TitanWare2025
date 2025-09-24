package frc.robot.utils.commands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

public class LoggedTrigger implements BooleanSupplier {
    /** Functional interface for the body of a trigger binding. */
    @FunctionalInterface
    private interface BindingBody {
        /**
         * Executes the body of the binding.
         *
         * @param previous The previous state of the condition.
         * @param current The current state of the condition.
         * @param schedule Consumer for the command(s) to schedule.
         * @param cancel Consumer for the command(s) to cancel.
         */
        void run(
                final boolean previous,
                final boolean current,
                final Consumer<Command> schedule,
                final Consumer<Command> cancel
        );
    }

    private String name;
    private final StringBuilder nameBuilder;

    private final BooleanSupplier condition;
    private final EventLoop loop;

    private LoggedTrigger(
            final String op,
            final StringBuilder b0,
            final StringBuilder b1,
            final EventLoop loop,
            final BooleanSupplier condition
    ) {
        this.nameBuilder = b0
                .append(op)
                .append("(")
                .append(b1)
                .append(")");

        this.loop = requireNonNullParam(loop, "loop", "LoggedTrigger");
        this.condition = requireNonNullParam(condition, "condition", "LoggedTrigger");
    }

    private LoggedTrigger(
            final String op,
            final StringBuilder nameBuilder,
            final EventLoop loop,
            final BooleanSupplier condition
    ) {
        this.nameBuilder = nameBuilder
                .insert(0, op);

        this.loop = requireNonNullParam(loop, "loop", "LoggedTrigger");
        this.condition = requireNonNullParam(condition, "condition", "LoggedTrigger");
    }

    /**
     * Creates a new trigger based on the given condition.
     *
     * @param name The name of the trigger
     * @param loop The loop instance that polls this trigger
     * @param condition The condition represented by this trigger
     */
    public LoggedTrigger(final String name, final EventLoop loop, final BooleanSupplier condition) {
        this.nameBuilder = new StringBuilder(name);

        this.loop = requireNonNullParam(loop, "loop", "LoggedTrigger");
        this.condition = requireNonNullParam(condition, "condition", "LoggedTrigger");
    }

    /**
     * Creates a new trigger based on the given condition.
     *
     * <p>Polled by the default scheduler button loop.
     *
     * @param name The name of the trigger
     * @param condition The condition represented by this trigger
     */
    public LoggedTrigger(final String name, final BooleanSupplier condition) {
        this(name, CommandScheduler.getInstance().getDefaultButtonLoop(), condition);
    }

    /**
     * Adds a binding to the EventLoop.
     *
     * @param body The body of the binding to add.
     */
    private void addBinding(final BindingBody body) {
        loop.bind(
                new Runnable() {
                    private boolean previous = condition.getAsBoolean();

                    @Override
                    public void run() {
                        final boolean current = condition.getAsBoolean();

                        body.run(
                                previous,
                                current,
                                command -> {

                                },
                                command -> {

                                });

                        previous = current;
                    }
                });
    }

    /**
     * Starts the command when the condition changes.
     *
     * @param command the command to start
     * @return this trigger, so calls can be chained
     */
    public LoggedTrigger onChange(Command command) {
        requireNonNullParam(command, "command", "onChange");
        addBinding(
                (previous,
                 current,
                 schedule,
                 cancel
                ) -> {
                    if (previous != current) {
                        schedule.accept(command);
                    }
                });
        return this;
    }

    /**
     * Starts the given command whenever the condition changes from `false` to `true`.
     *
     * @param command the command to start
     * @return this trigger, so calls can be chained
     */
    public LoggedTrigger onTrue(Command command) {
        requireNonNullParam(command, "command", "onTrue");
        addBinding(
                (previous,
                 current,
                 schedule,
                 cancel
                ) -> {
                    if (!previous && current) {
                        schedule.accept(command);
                    }
                });
        return this;
    }

    /**
     * Starts the given command whenever the condition changes from `true` to `false`.
     *
     * @param command the command to start
     * @return this trigger, so calls can be chained
     */
    public LoggedTrigger onFalse(Command command) {
        requireNonNullParam(command, "command", "onFalse");
        addBinding(
                (previous,
                 current,
                 schedule,
                 cancel
                ) -> {
                    if (previous && !current) {
                        schedule.accept(command);
                    }
                });
        return this;
    }

    /**
     * Starts the given command when the condition changes to `true` and cancels it when the condition
     * changes to `false`.
     *
     * <p>Doesn't re-start the command if it ends while the condition is still `true`. If the command
     * should restart, see {@link edu.wpi.first.wpilibj2.command.RepeatCommand}.
     *
     * @param command the command to start
     * @return this trigger, so calls can be chained
     */
    public LoggedTrigger whileTrue(Command command) {
        requireNonNullParam(command, "command", "whileTrue");
        addBinding(
                (previous,
                 current,
                 schedule,
                 cancel
                ) -> {
                    if (!previous && current) {
                        schedule.accept(command);
                    } else if (previous && !current) {
                        cancel.accept(command);
                    }
                });
        return this;
    }

    /**
     * Starts the given command when the condition changes to `false` and cancels it when the
     * condition changes to `true`.
     *
     * <p>Doesn't re-start the command if it ends while the condition is still `false`. If the command
     * should restart, see {@link edu.wpi.first.wpilibj2.command.RepeatCommand}.
     *
     * @param command the command to start
     * @return this trigger, so calls can be chained
     */
    public LoggedTrigger whileFalse(Command command) {
        requireNonNullParam(command, "command", "whileFalse");
        addBinding(
                (previous,
                 current,
                 schedule,
                 cancel
                ) -> {
                    if (previous && !current) {
                        schedule.accept(command);
                    } else if (!previous && current) {
                        cancel.accept(command);
                    }
                });
        return this;
    }

    /**
     * Toggles a command when the condition changes from `false` to `true`.
     *
     * @param command the command to toggle
     * @return this trigger, so calls can be chained
     */
    public LoggedTrigger toggleOnTrue(Command command) {
        requireNonNullParam(command, "command", "toggleOnTrue");
        addBinding(
                (previous,
                 current,
                 schedule,
                 cancel
                ) -> {
                    if (!previous && current) {
                        if (command.isScheduled()) {
                            cancel.accept(command);
                        } else {
                            schedule.accept(command);
                        }
                    }
                });
        return this;
    }

    /**
     * Toggles a command when the condition changes from `true` to `false`.
     *
     * @param command the command to toggle
     * @return this trigger, so calls can be chained
     */
    public LoggedTrigger toggleOnFalse(Command command) {
        requireNonNullParam(command, "command", "toggleOnFalse");
        addBinding(
                (previous,
                 current,
                 schedule,
                 cancel
                ) -> {
                    if (previous && !current) {
                        if (command.isScheduled()) {
                            cancel.accept(command);
                        } else {
                            schedule.accept(command);
                        }
                    }
                });
        return this;
    }

    public String getName() {
        if (name == null) {
            name = nameBuilder.toString();
        }

        return name;
    }

    @Override
    public String toString() {
        return getName();
    }

    @Override
    public boolean getAsBoolean() {
        return condition.getAsBoolean();
    }

    /**
     * Composes two triggers with logical AND.
     *
     * @param trigger the condition to compose with
     * @return A trigger which is active when both component triggers are active.
     */
    public LoggedTrigger and(final LoggedTrigger trigger) {
        return new LoggedTrigger(
                "and",
                nameBuilder,
                trigger.nameBuilder,
                loop,
                () -> condition.getAsBoolean() && trigger.getAsBoolean()
        );
    }

    /**
     * Composes two triggers with logical OR.
     *
     * @param trigger the condition to compose with
     * @return A trigger which is active when either component trigger is active.
     */
    public LoggedTrigger or(final LoggedTrigger trigger) {
        return new LoggedTrigger(
                "and",
                nameBuilder,
                trigger.nameBuilder,
                loop,
                () -> condition.getAsBoolean() || trigger.getAsBoolean()
        );
    }

    /**
     * Creates a new trigger that is active when this trigger is inactive, i.e. that acts as the
     * negation of this trigger.
     *
     * @return the negated trigger
     */
    public LoggedTrigger negate() {
        return new LoggedTrigger(
                "!",
                nameBuilder,
                loop,
                () -> !condition.getAsBoolean()
        );
    }

    /**
     * Creates a new debounced trigger from this trigger - it will become active when this trigger has
     * been active for longer than the specified period.
     *
     * @param seconds The debounce period.
     * @return The debounced trigger (rising edges debounced only)
     */
    public LoggedTrigger debounce(final double seconds) {
        return debounce(seconds, Debouncer.DebounceType.kRising);
    }

    /**
     * Creates a new debounced trigger from this trigger - it will become active when this trigger has
     * been active for longer than the specified period.
     *
     * @param seconds The debounce period.
     * @param type The debounce type.
     * @return The debounced trigger.
     */
    public LoggedTrigger debounce(final double seconds, final Debouncer.DebounceType type) {
        return new LoggedTrigger(
                String.format("(%.2fs)", seconds),
                loop,
                new BooleanSupplier() {
                    final Debouncer debouncer = new Debouncer(seconds, type);

                    @Override
                    public boolean getAsBoolean() {
                        return debouncer.calculate(condition.getAsBoolean());
                    }
                });
    }
}
