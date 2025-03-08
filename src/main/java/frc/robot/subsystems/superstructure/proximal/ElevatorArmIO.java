package frc.robot.subsystems.superstructure.proximal;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorArmIO {
    @AutoLog
    class ElevatorArmIOInputs {
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
     * @see ElevatorArmIOInputs
     * @see AutoLog
     */
    default void updateInputs(final ElevatorArmIOInputs inputs) {}

    default void config() {}

    default void toPivotPosition(final double pivotPositionRots) {}

    default void toPivotVoltage(final double pivotVolts) {}

    default void toPivotTorqueCurrent(final double pivotTorqueCurrentAmps) {}
}
