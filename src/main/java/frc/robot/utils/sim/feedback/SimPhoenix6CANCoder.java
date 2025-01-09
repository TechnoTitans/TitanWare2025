package frc.robot.utils.sim.feedback;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.sim.CANcoderSimState;
import frc.robot.utils.ctre.Phoenix6Utils;

public class SimPhoenix6CANCoder implements SimFeedbackSensor {
    private final CANcoder canCoder;
    private final CANcoderSimState simState;

    public SimPhoenix6CANCoder(final CANcoder canCoder) {
        this.canCoder = canCoder;
        this.simState = canCoder.getSimState();
    }

    @Override
    public void setSupplyVoltage(double volts) {
        Phoenix6Utils.reportIfNotOk(canCoder, simState.setSupplyVoltage(volts));
    }

    @Override
    public void setRawPosition(double rotations) {
        Phoenix6Utils.reportIfNotOk(canCoder, simState.setRawPosition(rotations));
    }

    @Override
    public void addPosition(double deltaRotations) {
        Phoenix6Utils.reportIfNotOk(canCoder, simState.addPosition(deltaRotations));
    }

    @Override
    public void setVelocity(double rotationsPerSec) {
        Phoenix6Utils.reportIfNotOk(canCoder, simState.setVelocity(rotationsPerSec));
    }
}
