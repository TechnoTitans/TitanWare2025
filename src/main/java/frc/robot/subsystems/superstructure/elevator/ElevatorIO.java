package frc.robot.subsystems.superstructure.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    class ElevatorIOInputs {
        public double masterPositionRots = 0.0;
        public double masterVelocityRotsPerSec = 0.0;
        public double masterVoltage = 0.0;
        public double masterTorqueCurrentAmps = 0.0;
        public double masterTempCelsius = 0.0;

        public double followerPositionRots = 0.0;
        public double followerVelocityRotsPerSec = 0.0;
        public double followerVoltage = 0.0;
        public double followerTorqueCurrentAmps = 0.0;
        public double followerTempCelsius = 0.0;

        public double encoderPositionRots = 0.0;
        public double encoderVelocityRotsPerSec = 0.0;

        public boolean magneticLimitSwitch = false;
    }

    /**
     * Updates the set of loggable inputs.
     * @param inputs Logged class of IOInputs
     * @see ElevatorIOInputs
     * @see AutoLog
     */
    default void updateInputs(final ElevatorIOInputs inputs) {}

    default void config() {}

    default void setPosition(final double positionRots) {}

    default void toPosition(final double positionRots) {}

    default void toVoltage(final double volts) {}

    default void toTorqueCurrent(final double torqueCurrentAmps) {}

    default void setLimitSwitchState(final boolean limitSwitchActive) {}
}
