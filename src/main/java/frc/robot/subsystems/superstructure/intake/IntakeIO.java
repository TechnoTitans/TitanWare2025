package frc.robot.subsystems.superstructure.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    class IntakeIOInputs {
        public double pivotPositionRots = 0.0;
        public double pivotVelocityRotsPerSec = 0.0;
        public double pivotVoltage = 0.0;
        public double pivotTorqueCurrentAmps = 0.0;
        public double pivotTempCelsius = 0.0;

        public double encoderPositionRots = 0.0;
        public double encoderVelocityRotsPerSec = 0.0;

        public double algaeRollerPositionRots = 0.0;
        public double algaeRollerVelocityRotsPerSec = 0.0;
        public double algaeRollerVoltage = 0.0;
        public double algaeRollerTorqueCurrentAmps = 0.0;
        public double algaeRollerTempCelsius = 0.0;

        public double coralRollerPositionRots = 0.0;
        public double coralRollerVelocityRotsPerSec = 0.0;
        public double coralRollerVoltage = 0.0;
        public double coralRollerTorqueCurrentAmps = 0.0;
        public double coralRollerTempCelsius = 0.0;

        public double CANRangeDistance = 0.0;
        public boolean CANRangeIsDetected = false;
    }

    /**
     * Updates the set of loggable inputs.
     * @param inputs Logged class of IOInputs
     * @see IntakeIOInputs
     * @see AutoLog
     */

    default void updateInputs(final IntakeIOInputs inputs) {}

    default void config() {}

    default void toPivotPosition(final double pivotPositionRots) {}

    default void pivotToVoltage(final double volts) {}

    default void pivotToTorqueCurrent(final double torqueCurrentAmps) {}

    default void setAlgaeRollerVelocity(final double algaeRollerVelocityRotsPerSec) {}

    default void algaeRollerToVoltage(final double volts) {}

    default void algaeRollerToTorqueCurrent(final double torqueCurrentAmps) {}

    default void setCoralRollerVelocity(final double coralRollerVelocityRotsPerSec) {}

    default void coralRollerToVoltage(final double volts) {}

    default void coralRollerToTorqueCurrent(final double torqueCurrentAmps) {}
}
