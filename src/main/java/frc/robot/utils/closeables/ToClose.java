package frc.robot.utils.closeables;

import edu.wpi.first.wpilibj.DriverStation;

import java.util.ArrayList;
import java.util.List;

public final class ToClose {
    private static boolean hasClosed = false;
    private static boolean hasHooked = false;

    private static final List<AutoCloseable> toClose = new ArrayList<>();
    private static final Thread closingThread = new Thread(() -> {
        if (hasClosed) {
            return;
        }
        hasClosed = true;

        for (final AutoCloseable autoCloseable : toClose) {
            try {
                autoCloseable.close();
            } catch (final Exception exception) {
                DriverStation.reportError(
                        String.format("Failed to close resource: %s", exception), exception.getStackTrace()
                );
            }
        }
    });

    private ToClose() {
    }

    public static void add(final AutoCloseable autoCloseable) {
        toClose.add(autoCloseable);
    }

    public static void hook() {
        if (hasHooked || hasClosed) {
            throw new RuntimeException("hook() can only be invoked once before closing!");
        }

        hasHooked = true;
        Runtime.getRuntime().addShutdownHook(closingThread);
    }

    public static boolean hasClosed() {
        return hasClosed;
    }

    public static boolean hasHooked() {
        return hasHooked;
    }
}
