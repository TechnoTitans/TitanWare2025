package frc.robot.subsystems.superstructure.distal;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeArmIO {
    @AutoLog
    class IntakeArmIOInputs {
        public double pivotPositionRots = 0.0;
        public double pivotVelocityRotsPerSec = 0.0;
        public double pivotVoltage = 0.0;
        public double pivotTorqueCurrentAmps = 0.0;
        public double pivotTempCelsius = 0.0;

        public double encoderPositionRots = 0.0;
        public double encoderVelocityRotsPerSec = 0.0;
    }

    /**
     * Updates the set of loggable inputs.
     * @param inputs Logged class of IOInputs
     * @see IntakeArmIOInputs
     * @see AutoLog
     */

    default void updateInputs(final IntakeArmIOInputs inputs) {}

    default void config() {}

    default void toPivotPosition(final double pivotPositionRots) {}

    default void toPivotPositionUnprofiled(
            final double pivotPositionRots,
            final double pivotVelocityRotsPerSec
    ) {}

    default void toPivotVoltage(final double volts) {}
}
