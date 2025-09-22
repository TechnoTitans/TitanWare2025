package frc.robot.utils.ctre;

import com.ctre.phoenix6.BaseStatusSignal;
import frc.robot.constants.HardwareConstants;

public class RefreshAll {
    private static BaseStatusSignal[] rioSignals = new BaseStatusSignal[0];
    private static BaseStatusSignal[] canivoreSignals = new BaseStatusSignal[0];

    public static void add(final HardwareConstants.CANBus bus, final BaseStatusSignal... signals) {
        final BaseStatusSignal[] existingSignals = switch (bus) {
            case RIO -> rioSignals;
            case CANIVORE -> canivoreSignals;
        };

        final int nExistingSignals = existingSignals.length;
        final int nNewSignals = signals.length;

        final BaseStatusSignal[] newSignals = new BaseStatusSignal[nExistingSignals + nNewSignals];
        System.arraycopy(existingSignals, 0, newSignals, 0, nExistingSignals);
        System.arraycopy(signals, 0, newSignals, nExistingSignals, nNewSignals);

        switch (bus) {
            case RIO -> rioSignals = newSignals;
            case CANIVORE -> canivoreSignals = newSignals;
        }
    }

    public static void refreshAll(final HardwareConstants.CANBus bus) {
        final BaseStatusSignal[] signals = switch (bus) {
            case RIO -> rioSignals;
            case CANIVORE -> canivoreSignals;
        };

        if (signals.length != 0) {
            BaseStatusSignal.refreshAll(signals);
        }
    }

    public static void refreshAll() {
        refreshAll(HardwareConstants.CANBus.RIO);
        refreshAll(HardwareConstants.CANBus.CANIVORE);
    }
}
