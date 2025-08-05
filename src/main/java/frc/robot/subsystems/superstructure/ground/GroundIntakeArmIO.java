package frc.robot.subsystems.superstructure.ground;

import org.littletonrobotics.junction.AutoLog;

public interface GroundIntakeArmIO {

    @AutoLog
    class GroundIntakeArmIOInputs {
        public double pivotPositionRots = 0.0;
        public double pivotVelocityRotsPerSec = 0.0;
        public double pivotVoltage = 0.0;
        public double pivotTorqueCurrentAmps = 0.0;
        public double pivotTempCelsius = 0.0;

        public double encoderPositionRots = 0.0;
        public double encoderVelocityRotsPerSec = 0.0;
    }

    default void updateInputs(final GroundIntakeArmIOInputs inputs) {}

    default void config() {}

    default void toPivotPosition(final double positionRots) {}

    default void toPivotVoltage(final double volts) {}

    default void toPivotTorqueCurrent(final double torqueCurrent) {}
}
