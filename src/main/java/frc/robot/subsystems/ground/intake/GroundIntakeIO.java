package frc.robot.subsystems.ground.intake;

import org.littletonrobotics.junction.AutoLog;

public interface GroundIntakeIO {

    @AutoLog
    public class GroundIntakeIOInputs {
        public double rollerPositionRots = 0.0;
        public double rollerVelocityRotsPerSec = 0.0;
        public double rollerVoltage = 0.0;
        public double rollerTorqueCurrentAmps = 0.0;
        public double rollerTempCelsius = 0.0;

        public double coralTOFDistanceMeters = 0.0;
    }

    /**
     * Updates the set of loggable inputs.
     * @param inputs Logged class of IOInputs
     * @see GroundIntakeIO.GroundIntakeIOInputs
     * @see AutoLog
     */

    default void updateInputs(final GroundIntakeIOInputs inputs) {}

    default void config() {}

    default void toRollerVelocity(final double velocityRotsPerSec) {}
    
    default void toRollerVoltage(final double volts) {}
    
    default void toRollerTorqueCurrent(final double torqueCurrentAmps) {}

    default void setTOFDistance(final double distanceMeters) {}
}
