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

    private static final Deque<Integer> traces = new ArrayDeque<>();

    private static final List<Long> timestamps = new ArrayList<>();
    private static final Map<Integer, String> names = new HashMap<>();
    private static final Map<Integer, Long> budgets = new HashMap<>();
    private static final Map<Integer, Long> durations = new HashMap<>();

    private static final SortedMap<Long, Integer> overrunTimestamps = new TreeMap<>();
    private static final SortedMap<Long, Integer> overrunStopAtTraceCount = new TreeMap<>();

    private static boolean running = false;
    private static int traceCount = 0;

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
            timestamps.add(RobotController.getFPGATime());
            names.put(traceCount, name);
            budgets.put(traceCount, (long) LogUtils.millisecondsToMicroseconds(budgetMs));

            traces.push(traceCount);
            traceCount++;
        }
    }

    public static void trace(final String name) {
        trace(name, 20);
    }

    public static void stop() {
        if (!running || traces.isEmpty()) {
            return;
        }

        final int traceAt = traces.pop();

        final long timestamp = timestamps.get(traceAt);
        final long now = RobotController.getFPGATime();
        final long duration = now - timestamp;

        durations.put(traceAt, duration);

        final long budget = budgets.get(traceAt);
        if (duration > budget) {
            overrunTimestamps.put(timestamp, traceAt);
            overrunStopAtTraceCount.put(timestamp, traceCount - 1);
        }
    }

    private static void cleanupOldTimestamps(
            final NavigableMap<Long, String> from,
            final long olderThanTimestamp
    ) {
        while (!from.isEmpty()) {
            final Map.Entry<Long, String> entry = from.firstEntry();
            final long timestamp = entry.getKey();
            if (timestamp <= olderThanTimestamp) {
                from.remove(timestamp);
            } else {
                break;
            }
        }
    }

    public static void periodic() {
        if (running) {
            alertTypePub.set(AlertType);

            final long now = RobotController.getFPGATime();
            if (!overrunTimestamps.isEmpty()) {
                final NavigableMap<Long, String> outNames = new TreeMap<>();
                final Collection<String> outNamesView = outNames.values();

                for (final Map.Entry<Long, Integer> timestampEntry : overrunTimestamps.entrySet()) {
                    final long timestamp = timestampEntry.getKey();
                    final int traceCount = timestampEntry.getValue();
                    final int stopAtTraceCount = overrunStopAtTraceCount.get(timestamp);

                    for (int i = traceCount; i <= stopAtTraceCount; i++) {
                        final long innerTimestamp = timestamps.get(i);
                        final long innerDuration = durations.get(i);
                        final String innerName = names.get(i);

                        outNames.put(innerTimestamp + innerDuration, innerName);
                        cleanupOldTimestamps(outNames, innerTimestamp);

                        warningsPub.set(outNamesView.toArray(EmptyStringArray), innerTimestamp);
                    }
                }

                final long lastTimestamp = outNames.lastKey();
                cleanupOldTimestamps(outNames, lastTimestamp);
                warningsPub.set(outNamesView.toArray(EmptyStringArray), lastTimestamp);
            } else {
                warningsPub.set(EmptyStringArray, now);
            }

            traces.clear();

            timestamps.clear();
            names.clear();
            budgets.clear();
            durations.clear();

            overrunTimestamps.clear();
            overrunStopAtTraceCount.clear();

            traceCount = 0;
        }
    }
}