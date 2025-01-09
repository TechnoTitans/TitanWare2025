package frc.robot.utils.sim.motors;

import frc.robot.utils.sim.feedback.SimFeedbackSensor;

public interface SimMotorController {
    /**
     * Attaches a {@link SimFeedbackSensor} to the simulated motor controller
     *
     * @param feedbackSensor the {@link SimFeedbackSensor}
     */
    void attachFeedbackSensor(final SimFeedbackSensor feedbackSensor);

    /**
     * Updates the motor controllers with time
     *
     * @param dtSeconds the amount of time since the last update call (in seconds)
     */
    void update(final double dtSeconds);

    /**
     * Updates the motor controllers directly (raw)
     *
     * @param mechanismPositionRots       the position of the simulated mechanism, in rotations
     * @param mechanismVelocityRotsPerSec the velocity of the simulated mechanism, in rotations/sec
     */
    void rawUpdate(final double mechanismPositionRots, final double mechanismVelocityRotsPerSec);

    /**
     * Position of the simulated motor (mechanism)
     *
     * @return the position, in rotations
     */
    double getAngularPositionRots();

    /**
     * Velocity of the simulated motor (mechanism)
     *
     * @return the velocity, in rotations/sec
     */
    double getAngularVelocityRotsPerSec();

    /**
     * Get the output voltage of the motor(s)
     *
     * @return the output voltage (in volts)
     */
    double getMotorVoltage();

    /**
     * Get the output current of the motor(s)
     *
     * @return the output current (in amps)
     */
    double getMotorCurrent();
}
