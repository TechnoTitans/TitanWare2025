package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    class IntakeIOInputs {
        public double coralRollerPositionRots = 0.0;
        public double coralRollerVelocityRotsPerSec = 0.0;
        public double coralRollerVoltage = 0.0;
        public double coralRollerTorqueCurrentAmps = 0.0;
        public double coralRollerTempCelsius = 0.0;

        public double algaeRollerPositionRots = 0.0;
        public double algaeRollerVelocityRotsPerSec = 0.0;
        public double algaeRollerVoltage = 0.0;
        public double algaeRollerTorqueCurrentAmps = 0.0;
        public double algaeRollerTempCelsius = 0.0;

        public double coralCANRangeDistanceMeters = 0.0;
        public boolean coralCANRangeIsDetected = false;
    }

    /**
     * Updates the set of loggable inputs.
     * @param inputs Logged class of IOInputs
     * @see IntakeIOInputs
     * @see AutoLog
     */

    default void updateInputs(final IntakeIOInputs inputs) {}

    default void config() {}

    default void toCoralRollerVelocity(final double coralRollerVelocityRotsPerSec) {}

    default void toCoralRollerVoltage(final double volts) {}

    default void toCoralRollerTorqueCurrent(final double torqueCurrentAmps) {}

    default void toAlgaeRollerVelocity(final double algaeRollerVelocityRotsPerSec) {}

    default void toAlgaeRollerVoltage(final double volts) {}

    default void toAlgaeRollerTorqueCurrent(final double torqueCurrentAmps) {}
}
