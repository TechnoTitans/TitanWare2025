package frc.robot.utils.sim.sensors;

import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.math.StateSpaceUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N1;

public class TimeOfFlight extends com.playingwithfusion.TimeOfFlight implements AutoCloseable {
    private final SimDevice simDevice;
    private SimDouble simDistanceMillimeters;

    private final Vector<N1> distStdDevs = VecBuilder.fill(10);

    public TimeOfFlight(final int canID) {
        super(canID);

        simDevice = SimDevice.create("CAN:TimeOfFlight", canID);
        if (simDevice != null) {
            simDistanceMillimeters = simDevice.createDouble("range", SimDevice.Direction.kInput, 0.0);
        }
    }

    @Override
    public void close() {
        if (simDevice != null) {
            simDevice.close();
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
            final double noise = StateSpaceUtil.makeWhiteNoiseVector(distStdDevs).get(0, 0);
            return simDistanceMillimeters.get() + noise;
        }

        return super.getRange();
    }

    public double getRangeMeters() {
        return getRange() / 1000.0;
    }
}
