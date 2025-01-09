package frc.robot.utils.logging;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import java.util.HashMap;
import java.util.Map;

/**
 * Class for a tunable number. Gets value from dashboard in tuning mode, returns default if not or
 * value not in dashboard.
 */
public class LoggedTunableNumber {
    private static final String tableKey = "TunableNumbers";

    private final LoggedNetworkNumber dashboardNumber;
    private final Map<Integer, Double> lastHasChangedValues = new HashMap<>();

    /**
     * Create a new LoggedTunableNumber with the default value
     *
     * @param dashboardKey Key on dashboard
     * @param defaultValue Default value
     */
    public LoggedTunableNumber(String dashboardKey, double defaultValue) {
        this.dashboardNumber = new LoggedNetworkNumber(
                tableKey + "/SmartDashboard" + dashboardKey,
                defaultValue
        );
    }

    /**
     * Create a new LoggedTunableNumber
     *
     * @param dashboardKey Key on dashboard
     */
    public LoggedTunableNumber(String dashboardKey) {
        this(tableKey + "/SmartDashboard" + dashboardKey, 0);
    }

    /**
     * Get the current value, from dashboard if available and in tuning mode.
     *
     * @return The current value
     */
    public double get() {
        return dashboardNumber.get();
    }

    /**
     * Checks whether the number has changed since our last check
     *
     * @param id Unique identifier for the caller to avoid conflicts when shared between multiple
     *           objects. Recommended approach is to pass the result of "hashCode()"
     * @return True if the number has changed since the last time this method was called, false
     * otherwise.
     */
    public boolean hasChanged(int id) {
        double currentValue = get();
        Double lastValue = lastHasChangedValues.get(id);
        if (lastValue == null || currentValue != lastValue) {
            lastHasChangedValues.put(id, currentValue);
            return true;
        }
        return false;
    }
}
