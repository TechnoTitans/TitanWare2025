package frc.robot.utils.logging;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringArrayPublisher;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.RobotController;

import java.util.*;

public class Tracer {
    private static final String LogKey = "Tracer";
    private static final String AlertType = "Alerts";

    private static final String[] EmptyStringArray = new String[0];

    private static final Deque<Long> timestamps = new LinkedList<>();
    private static final Map<Integer, String> names = new HashMap<>();
    private static final Map<Integer, Long> budgets = new HashMap<>();
    private static final Map<String, Long> durations = new HashMap<>();

    private static final SortedMap<Long, String> outTimestamps = new TreeMap<>();
    private static final Map<String, Long> outDurations = new HashMap<>();

    private static boolean running = false;
    private static int level = 0;

    private static StringPublisher alertTypePub;
    private static StringArrayPublisher warningsPub;

    public static void init() {
        if (!running) {
            running = true;

            final NetworkTable table = NetworkTableInstance.getDefault().getTable(LogKey);
            alertTypePub = table.getStringTopic(".type").publish();
            warningsPub = table.getStringArrayTopic("warnings").publish();
        }
    }

    public static void trace(final String name, final double budgetMs) {
        if (running) {
            timestamps.addLast(RobotController.getFPGATime());
            names.put(level, name);
            budgets.put(level, (long)LogUtils.millisecondsToMicroseconds(budgetMs));
            level++;
        }
    }

    public static void trace(final String name) {
        trace(name, 20);
    }

    public static void stop() {
        if (!running || level <= 0) {
            return;
        }

        final int at = level - 1;
        final String name = names.get(at);
        names.remove(at);

        final long timestamp = timestamps.getLast();
        timestamps.removeLast();

        final long now = RobotController.getFPGATime();
        final long duration = now - timestamp;
        durations.put(name, duration);

        final long budget = budgets.get(at);
        if (duration > budget) {
            outTimestamps.put(timestamp, name);
            outDurations.put(name, duration);
        }

        level = at;
    }

    public static void periodic() {
        if (running) {
            alertTypePub.set(AlertType);

            if (!outTimestamps.isEmpty()) {
                final NavigableMap<Long, String> names = new TreeMap<>();
                for (final Map.Entry<Long, String> timestampEntry : outTimestamps.entrySet()) {
                    final String name = timestampEntry.getValue();
                    final long timestamp = timestampEntry.getKey();
                    final long duration = outDurations.get(name);

                    names.put(timestamp + duration, name);
                    while (!names.isEmpty()) {
                        final Map.Entry<Long, String> entry = names.firstEntry();
                        final long removeTimestamp = entry.getKey();
                        if (timestamp > removeTimestamp) {
                            names.remove(removeTimestamp);
                        } else {
                            break;
                        }
                    }

                    warningsPub.set(names.values().toArray(new String[0]), timestamp);
                }
            } else {
                warningsPub.set(EmptyStringArray);
            }

            outTimestamps.clear();
            outDurations.clear();
        }
    }
}
