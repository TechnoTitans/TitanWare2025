package frc.robot.subsystems.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
    @AutoLog
    class GyroIOInputs {
        /**
         * Reports the current state of the Gyro, in particular, if it has a hardware fault.
         * True if it has a hardware fault, false if not.
         */
        public boolean hasHardwareFault = false;

        /**
         * The gyro reported roll position (in absolute [-180, 180) degrees)
         */
        public double rollPositionDeg = 0.0;
        /**
         * The gyro reported pitch position (in absolute [-90, 90) degrees)
         */
        public double pitchPositionDeg = 0.0;
        /**
         * The gyro reported yaw position (in absolute [-368640, 368640) degrees, which is 1024 rotations)
         */
        public double yawPositionDeg = 0.0;

        /**
         * The gyro reported roll (Y-axis) velocity (in absolute [-1999, 1999) deg/sec)
         */
        public double rollVelocityDegPerSec = 0.0;
        /**
         * The gyro reported pitch (X-axis) velocity (in absolute [-1999, 1999) deg/sec)
         */
        public double pitchVelocityDegPerSec = 0.0;
        /**
         * The gyro reported yaw (Z-axis) velocity (in absolute [-1999, 1999) deg/sec)
         */
        public double yawVelocityDegPerSec = 0.0;

        public double[] odometryTimestampsSec = new double[]{};
        public double[] odometryYawPositionsDeg = new double[]{};
    }

    /**
     * Updates the set of loggable inputs.
     *
     * @param inputs Logged class of IOInputs
     * @see GyroIOInputs
     * @see AutoLog
     */
    default void updateInputs(final GyroIOInputs inputs) {
    }

    /**
     * Periodic call to update the {@link GyroIO}.
     */
    default void periodic() {
    }

    /**
     * Call to configure the Pigeon, should only be called once on init
     */
    default void config() {
    }

    /**
     * Set the currently observed angle of the Gyro
     *
     * @param angle the angle to set (deg)
     */
    default void setAngle(final Rotation2d angle) {
    }

    /**
     * Sets the current observed angle of the Gyro to 0 (deg)
     *
     * @see GyroIO#setAngle(Rotation2d)
     */
    default void zeroRotation() {
        setAngle(Rotation2d.fromDegrees(0));
    }
}
