package frc.robot.utils.sim.feedback;

public interface SimFeedbackSensor {
    /**
     * Sets the simulated supply voltage of the {@link SimFeedbackSensor}.
     *
     * @param volts the supply voltage in Volts
     */
    void setSupplyVoltage(final double volts);

    /**
     * Sets the simulated raw position of the {@link SimFeedbackSensor}.
     *
     * @param rotations the raw position in rotations
     */
    void setRawPosition(final double rotations);

    /**
     * Adds to the simulated position of the {@link SimFeedbackSensor}.
     *
     * @param deltaRotations The change in position in rotations
     */
    void addPosition(final double deltaRotations);

    /**
     * Sets the simulated velocity of the {@link SimFeedbackSensor}.
     *
     * @param rotationsPerSec The new velocity in rotations per second
     */
    void setVelocity(final double rotationsPerSec);
}
