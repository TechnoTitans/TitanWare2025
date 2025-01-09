package frc.robot.utils.sim;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.core.CoreTalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.sim.ChassisReference;

/**
 * Simulation shared utility methods/functions
 */
public class SimUtils {
    public static ChassisReference invertedValueToChassisReference(final InvertedValue invertedValue) {
        return switch (invertedValue) {
            case Clockwise_Positive -> ChassisReference.Clockwise_Positive;
            case CounterClockwise_Positive -> ChassisReference.CounterClockwise_Positive;
        };
    }

    public static ChassisReference sensorDirectionToChassisReference(final SensorDirectionValue sensorDirectionValue) {
        return switch (sensorDirectionValue) {
            case Clockwise_Positive -> ChassisReference.Clockwise_Positive;
            case CounterClockwise_Positive -> ChassisReference.CounterClockwise_Positive;
        };
    }

    public static void setCTRETalonFXSimStateMotorInverted(
            final CoreTalonFX talonFX,
            final InvertedValue invertedValue
    ) {
        talonFX.getSimState().Orientation = invertedValueToChassisReference(invertedValue);
    }

    public static void setCTRECANCoderSimStateSensorDirection(
            final CANcoder canCoder,
            final SensorDirectionValue sensorDirectionValue
    ) {
        canCoder.getSimState().Orientation = sensorDirectionToChassisReference(sensorDirectionValue);
    }

    /**
     * Initialize a simulated relative {@link CANcoder}.
     *
     * <p>
     * There is a known issue where a {@link CANcoder} defaults to a position of 0.25 in simulation.
     * This wouldn't matter much if the {@link CANcoder} was used as a relative sensor,
     * but it can affect its use as an absolute sensor.
     * For now, I think the workaround is to call {@link CANcoder#setPosition(double)} with a position of 0
     * on the {@link CANcoder} at startup in simulation (wrap in a {@link Utils#isSimulation()} check).
     * </p>
     *
     * @param canCoder the {@link CANcoder}
     * @return the {@link StatusCode} returned from the {@link CANcoder#setPosition(double)} call
     */
    @SuppressWarnings("UnusedReturnValue")
    public static StatusCode initializeCTRECANCoderSim(final CANcoder canCoder) {
        if (!Utils.isSimulation()) {
            throw new RuntimeException(
                    "Cannot initialize a simulated relative CANCoder when not in sim! (This is likely a bug!)"
            );
        }
        // TODO: this doesn't seem to work for all CANCoders
        return canCoder.setPosition(0);
    }

    /**
     * Applies the effects of friction to dampen the motor voltage.
     *
     * @param motorVoltage    Voltage output by the motor
     * @param frictionVoltage Voltage required to overcome friction
     * @return Friction-dampened motor voltage
     */
    public static double addMotorFriction(final double motorVoltage, final double frictionVoltage) {
        return Math.abs(motorVoltage) < frictionVoltage
                ? 0
                : motorVoltage - Math.copySign(frictionVoltage, motorVoltage);
    }
}
