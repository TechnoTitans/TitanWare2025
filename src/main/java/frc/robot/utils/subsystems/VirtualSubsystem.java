package frc.robot.utils.subsystems;

import edu.wpi.first.wpilibj.Notifier;
import frc.robot.constants.Constants;
import frc.robot.utils.closeables.ToClose;

import java.util.HashMap;
import java.util.HashSet;
import java.util.Set;

public class VirtualSubsystem {
    private static final Set<VirtualSubsystem> loopPeriodSubsystems = new HashSet<>();
    private static final HashMap<VirtualSubsystem, Notifier> subsystemNotifierMap = new HashMap<>();

    public VirtualSubsystem(final double loopPeriodSeconds) {
        if (loopPeriodSeconds != Constants.LOOP_PERIOD_SECONDS) {
            final Notifier subsystemNotifier = new Notifier(this::periodic);
            ToClose.add(subsystemNotifier);

            subsystemNotifier.startPeriodic(loopPeriodSeconds);
            VirtualSubsystem.subsystemNotifierMap.put(this, subsystemNotifier);
        } else {
            VirtualSubsystem.loopPeriodSubsystems.add(this);
        }
    }

    public VirtualSubsystem() {
        this(Constants.LOOP_PERIOD_SECONDS);
    }

    /**
     * Periodic call of the {@link VirtualSubsystem}.
     */
    public void periodic() {
        throw new IllegalStateException("Non implemented periodic() should not exist!");
    }

    /**
     * Runs all virtual subsystems that are bound to the loop cycle
     * (has a loop period of {@link Constants#LOOP_PERIOD_SECONDS}).
     * <p>Should be called in {@link frc.robot.Robot#robotPeriodic()}</p>
     */
    public static void run() {
        for (final VirtualSubsystem subsystem : loopPeriodSubsystems) {
            subsystem.periodic();
        }
    }

    /**
     * Get the {@link VirtualSubsystem}s running on the {@link Constants#LOOP_PERIOD_SECONDS}, this
     * returns an internal mutable reference {@link Set}, so it should not be modified.
     *
     * @return the {@link Set} of registered {@link VirtualSubsystem}s that run periodically on the loop period
     */
    @SuppressWarnings("unused")
    public static Set<VirtualSubsystem> getLoopPeriodSubsystems() {
        return loopPeriodSubsystems;
    }

    /**
     * Get the {@link Notifier} of a registered {@link VirtualSubsystem} that has a
     * non-standard loop period != {@link Constants#LOOP_PERIOD_SECONDS}
     *
     * @param subsystem the {@link VirtualSubsystem} to get the {@link Notifier} for
     * @return the {@link Notifier} associated with the {@link VirtualSubsystem}
     */
    @SuppressWarnings("unused")
    public static Notifier getNonLoopPeriodNotifier(final VirtualSubsystem subsystem) {
        final Notifier notifier = subsystemNotifierMap.get(subsystem);
        if (notifier == null) {
            throw new RuntimeException(
                    "Attempted to get Notifier for non-registered or non-notifier VirtualSubsystem!"
            );
        }

        return notifier;
    }
}
