package frc.robot.subsystems.superstructure.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
    @AutoLog
    class ArmIOInputs {
        double pivotPositionRots = 0.0;
        double pivotVelocityRotsPerSec = 0.0;
        double pivotVoltageVolts = 0.0;
        double pivotTorqueCurrentAmps = 0.0;
        double pivotTempCelsius = 0.0;

        double pivotEncoderPositionRots = 0.0;
        double pivotEncoderVelocityRotsPerSec = 0.0;
    }

    /**
     * Updates the set of loggable inputs.
     * @param inputs Logged class of IOInputs
     * @see ArmIOInputs
     * @see AutoLog
     */
    default void updateInputs(final ArmIOInputs inputs) {}

    default void config() {}

    default void toPivotPosition(final double pivotPositionRots) {}

    default void toPivotVoltage(final double pivotVolts) {}

    default void toPivotTorqueCurrent(final double pivotTorqueCurrentAmps) {}
}
