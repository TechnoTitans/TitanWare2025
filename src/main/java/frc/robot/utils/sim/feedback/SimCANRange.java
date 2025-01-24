package frc.robot.utils.sim.feedback;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.CANrangeSimState;
import frc.robot.utils.ctre.Phoenix6Utils;

public class SimCANRange implements SimFeedbackSensor {
    private final CANrange canRange;
    private final CANrangeSimState simState;

    private final double linearMechanismDrumCircumferenceMeters;

    public SimCANRange(final CANrange canRange, final double linearMechanismDrumCircumferenceMeters) {
        this.canRange = canRange;
        this.simState = canRange.getSimState();

        this.linearMechanismDrumCircumferenceMeters = linearMechanismDrumCircumferenceMeters;
    }

    @Override
    public void setSupplyVoltage(double volts) {
        Phoenix6Utils.reportIfNotOk(canRange, simState.setSupplyVoltage(volts));
    }

    @Override
    public void setRawPosition(double rotations) {
        Phoenix6Utils.reportIfNotOk(canRange, simState.setDistance(rotations * linearMechanismDrumCircumferenceMeters));
    }
}
