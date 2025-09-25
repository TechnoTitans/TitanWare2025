package frc.robot.utils.commands;

import edu.wpi.first.wpilibj.DriverStation;

@SuppressWarnings("unused")
public class RobotModeLoggedTriggers {
    private RobotModeLoggedTriggers() {}

    /**
     * Returns a trigger that is true when the robot is enabled in autonomous mode.
     *
     * @return A trigger that is true when the robot is enabled in autonomous mode.
     */
    public static LoggedTrigger autonomous(final LoggedTrigger.Group group) {
        return group.t("autonomous", DriverStation::isAutonomousEnabled);
    }

    /**
     * Returns a trigger that is true when the robot is enabled in teleop mode.
     *
     * @return A trigger that is true when the robot is enabled in teleop mode.
     */
    public static LoggedTrigger teleop(final LoggedTrigger.Group group) {
        return group.t("teleop", DriverStation::isTeleopEnabled);
    }

    /**
     * Returns a trigger that is true when the robot is disabled.
     *
     * @return A trigger that is true when the robot is disabled.
     */
    public static LoggedTrigger disabled(final LoggedTrigger.Group group) {
        return group.t("disabled", DriverStation::isDisabled);
    }

    /**
     * Returns a trigger that is true when the robot is enabled in test mode.
     *
     * @return A trigger that is true when the robot is enabled in test mode.
     */
    public static LoggedTrigger test(final LoggedTrigger.Group group) {
        return group.t("test", DriverStation::isTestEnabled);
    }
}
