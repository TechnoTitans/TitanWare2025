package frc.robot.subsystems.intake.ground;

import org.littletonrobotics.junction.AutoLog;

public interface GroundIntakeIO {
    @AutoLog
    class GroundIntakeIOInputs {
        public double wheelPositionRots = 0.0;
        public double wheelVelocityRotsPerSec = 0.0;
        public double wheelVoltage = 0.0;
        public double wheelTorqueCurrentAmps = 0.0;
        public double wheelTempCelsius = 0.0;

        public double coralCANRangeDistanceMeters = 0.0;
        public boolean coralDetected = false;
    }

    /**
     * Updates the set of loggable inputs.
     * @param inputs Logged class of IOInputs
     * @see GroundIntakeIO.GroundIntakeIOInputs
     * @see AutoLog
     */

    default void updateInputs(final GroundIntakeIO.GroundIntakeIOInputs inputs) {}

    default void config() {}

    default void toWheelVelocity(final double velocityRotsPerSec) {}

    default void toWheelVoltage(final double volts) {}

    default void toWheelTorqueCurrent(final double torqueCurrentAmps) {}

    default void setCoralCANRangeDistance(final double distanceMeters) {}
}
