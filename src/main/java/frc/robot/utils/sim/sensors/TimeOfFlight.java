package frc.robot.utils.sim.sensors;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDouble;

public class TimeOfFlight extends com.playingwithfusion.TimeOfFlight {
    private final SimDevice simDevice;
    private SimDouble simDistanceMillimeters;

    public TimeOfFlight(final int canID) {
        super(canID);

        simDevice = SimDevice.create("CAN:TimeOfFlight", canID);
        if (simDevice != null) {
            simDistanceMillimeters = simDevice.createDouble("range", SimDevice.Direction.kInput, 0.0);
        }
    }

    public void setSimDistanceMillimeters(final double distanceMillimeters) {
        if (simDistanceMillimeters != null) {
            simDistanceMillimeters.set(distanceMillimeters);
        }
    }

    public void setSimDistanceMeters(final double distanceMeters) {
        setSimDistanceMillimeters(distanceMeters * 1000.0);
    }

    public double getRange() {
        if (simDistanceMillimeters != null) {
            return simDistanceMillimeters.get();
        }

        return super.getRange();
    }

    public double getRangeMeters() {
        return getRange() / 1000.0;
    }
}
