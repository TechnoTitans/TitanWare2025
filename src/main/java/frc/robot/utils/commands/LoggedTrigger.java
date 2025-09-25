package frc.robot.utils.commands;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.utils.logging.LoggedCommandScheduler;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

@SuppressWarnings({"UnusedReturnValue", "unused"})
public class LoggedTrigger implements BooleanSupplier {
    @SuppressWarnings("ClassCanBeRecord")
    public static class Group {
        private final String name;
        private final EventLoop loop;

        private Group(final String name, final EventLoop loop) {
            this.name = name;
            this.loop = loop;
        }

        public LoggedTrigger t(final String t, final BooleanSupplier condition) {
            return new LoggedTrigger(this, name + "." + t, loop, condition);
        }

        private LoggedTrigger t(final BooleanSupplier condition) {
            return new LoggedTrigger(this, name, loop, condition);
        }

        public static Group from(final String name, final EventLoop loop) {
            return new Group(name, loop);
        }

        public static Group from(final String name) {
            return from(name, CommandScheduler.getInstance().getDefaultButtonLoop());
        }
    }

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

    private static final int ItemsPerDescriptorLine = 3;
    private static int getDescriptorLineCount(final LoggedTrigger trigger) {
        return Math.floorDiv(trigger.names.length, ItemsPerDescriptorLine);
    }

    private static int LoggedTriggerId = 0;
    private static int nextId() {
        return LoggedTriggerId++;
    }

    public final int id;
    private final Group group;

    private String[] descriptor;
    private final String[] names;

    private final BooleanSupplier condition;
    private final EventLoop loop;

    /**
     * Creates a new trigger based on the given condition.
     *
     * @param group The group that created the trigger
     * @param name The name of the trigger
     * @param loop The loop instance that polls this trigger
     * @param condition The condition represented by this trigger
     */
    private LoggedTrigger(final Group group, final String name, final EventLoop loop, final BooleanSupplier condition) {
        this.id = nextId();
        this.group = group;
        this.names = new String[] {"", name, ""};

        this.loop = requireNonNullParam(loop, "loop", "LoggedTrigger");
        this.condition = requireNonNullParam(condition, "condition", "LoggedTrigger");
    }

    /**
     * Creates a new trigger based on the given condition.
     *
     * <p>Polled by the default scheduler button loop.
     *
     * @param group The group that created the trigger
     * @param name The name of the trigger
     * @param condition The condition represented by this trigger
     */
    private LoggedTrigger(final Group group, final String name, final BooleanSupplier condition) {
        this(group, name, CommandScheduler.getInstance().getDefaultButtonLoop(), condition);
    }

    private LoggedTrigger(
            final String op,
            final LoggedTrigger trigger,
            final EventLoop loop,
            final BooleanSupplier condition
    ) {
        final String[] tNames = trigger.names;
        final int tNamesLen = tNames.length;
        final String[] names = new String[tNamesLen];
        if (getDescriptorLineCount(trigger) <= 1) {
            names[0] = op;
            System.arraycopy(tNames, 1, names, 1, tNamesLen - 1);
        } else {
            names[0] = op + "(";
            System.arraycopy(tNames, 1, names, 1, tNamesLen - 1);
            names[tNamesLen - 1] += ")";
        }

        this.id = nextId();
        this.group = trigger.group;
        this.names = names;

        this.loop = requireNonNullParam(loop, "loop", "LoggedTrigger");
        this.condition = requireNonNullParam(condition, "condition", "LoggedTrigger");
    }

    private LoggedTrigger(
            final String op,
            final LoggedTrigger trigger,
            final LoggedTrigger other,
            final EventLoop loop,
            final BooleanSupplier condition
    ) {
        final String[] tNames = trigger.names;
        final String[] oNames = other.names;

        final int tNamesLen = tNames.length;
        final int oNamesLen = oNames.length;
        final String[] names = new String[tNamesLen + oNamesLen];

        System.arraycopy(tNames, 0, names, 0, tNamesLen);
        names[tNamesLen] = "." + op + "(" + oNames[0];
        System.arraycopy(oNames, 1, names, tNamesLen + 1, oNamesLen - 1);
        names[names.length - 1] += ")";

        this.id = nextId();
        this.group = trigger.group;
        this.names = names;

        this.loop = requireNonNullParam(loop, "loop", "LoggedTrigger");
        this.condition = requireNonNullParam(condition, "condition", "LoggedTrigger");
    }

    private LoggedTrigger(
            final String op,
            final LoggedTrigger trigger,
            final BooleanSupplier other,
            final EventLoop loop,
            final BooleanSupplier condition
    ) {
        this(op, trigger, trigger.group.t(other), loop, condition);
    }

    /**
     * Adds a binding to the EventLoop.
     *
     * @param body The body of the binding to add.
     */
    private void addBinding(final BindingBody body) {
        final LoggedTrigger trigger = this;
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
                                    LoggedCommandScheduler.scheduledBy(command, trigger);
                                    command.schedule();
                                },
                                Command::cancel
                        );

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

    public String[] getDescriptor() {
        if (descriptor != null) {
            return descriptor;
        }

        final int nNames = getDescriptorLineCount(this);
        final String[] descriptor = new String[nNames];
        if (nNames == 1) {
            descriptor[0] = names[1];
        } else {
            for (int i = 0; i < nNames; i++) {
                final int per = i * ItemsPerDescriptorLine;
                final String prefix = names[per];
                final String name = names[per + 1];
                final String suffix = names[per + 2];

                descriptor[i] = prefix
                        + name
                        + suffix;
            }
        }

        this.descriptor = descriptor;
        return descriptor;
    }

    @Override
    public boolean getAsBoolean() {
        return condition.getAsBoolean();
    }

    /**
     * Composes two triggers with logical AND.
     *
     * @param trigger the trigger to compose with
     * @return A trigger which is active when both component triggers are active.
     */
    public LoggedTrigger and(final LoggedTrigger trigger) {
        return new LoggedTrigger(
                "and",
                this,
                trigger,
                loop,
                () -> condition.getAsBoolean() && trigger.getAsBoolean()
        );
    }

    /**
     * Composes two triggers with logical AND.
     *
     * @param condition the condition to compose with
     * @return A trigger which is active when both component triggers are active.
     */
    public LoggedTrigger and(final BooleanSupplier condition) {
        return new LoggedTrigger(
                "and",
                this,
                condition,
                loop,
                () -> this.condition.getAsBoolean() && condition.getAsBoolean()
        );
    }


    /**
     * Composes two triggers with logical OR.
     *
     * @param trigger the trigger to compose with
     * @return A trigger which is active when either component trigger is active.
     */
    public LoggedTrigger or(final LoggedTrigger trigger) {
        return new LoggedTrigger(
                "or",
                this,
                trigger,
                loop,
                () -> condition.getAsBoolean() || trigger.getAsBoolean()
        );
    }

    /**
     * Composes two triggers with logical OR.
     *
     * @param condition the condition to compose with
     * @return A trigger which is active when either component trigger is active.
     */
    public LoggedTrigger or(final BooleanSupplier condition) {
        return new LoggedTrigger(
                "or",
                this,
                condition,
                loop,
                () -> this.condition.getAsBoolean() || condition.getAsBoolean()
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
                this,
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
                this,
                loop,
                new BooleanSupplier() {
                    private final Debouncer debouncer = new Debouncer(seconds, type);

                    @Override
                    public boolean getAsBoolean() {
                        return debouncer.calculate(condition.getAsBoolean());
                    }
                });
    }
}
