package frc.robot.subsystems.superstructure.arm.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeArmIO {
    @AutoLog
    class IntakeArmIOInputs {
        public double pivotPositionRots = 0.0;
        public double pivotVelocityRotsPerSec = 0.0;
        public double pivotAccelerationRotsPerSec2 = 0.0;
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

    default void toPivotVoltage(final double volts) {}
}
