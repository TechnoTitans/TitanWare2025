package frc.robot.subsystems.drive;

import com.ctre.phoenix6.*;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.ParentDevice;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.units.Measure;
import edu.wpi.first.util.DoubleCircularBuffer;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.Timer;

import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.locks.ReentrantReadWriteLock;
import java.util.function.Consumer;

public class OdometryThreadRunner {
    // Increase the priority to dedicate more resources towards running the thread at the right frequency, 1 is the
    // minimum realtime priority and should work well enough
    protected static final int STARTING_THREAD_PRIORITY = 1;
    // 250Hz should work well when on a CAN-FD network, if only on CAN 2.0 or other, this should probably be
    // set to <= 100Hz
    protected static final double UPDATE_FREQUENCY_HZ = 250;

    protected final ReentrantReadWriteLock signalQueueReadWriteLock;
    protected final ReentrantReadWriteLock signalReadWriteLock = new ReentrantReadWriteLock();
    protected final ReentrantReadWriteLock controlReqReadWriteLock = new ReentrantReadWriteLock();

    private String network;
    private CANBus canBus;

    protected final List<StatusSignal<?>> allSignals = new ArrayList<>();
    protected final Map<Long, ControlRequest> outerAppliedControlRequests = new HashMap<>();
    protected final Map<Long, ControlRequest> innerAppliedControlRequests = new HashMap<>();
    protected final Map<Long, Consumer<ControlRequest>> controlReqAppliers = new HashMap<>();
    protected final List<DoubleCircularBuffer> buffers = new ArrayList<>();
    protected final List<DoubleCircularBuffer> timestampBuffers = new ArrayList<>();

    protected final Thread thread;
    protected final State state = new State();
    protected volatile boolean running = false;

    protected final MedianFilter peakRemover = new MedianFilter(3);
    protected final LinearFilter lowPass = LinearFilter.movingAverage(50);
    protected double lastTimeSeconds = 0;
    protected double currentTimeSeconds = 0;
    protected double averageLoopTimeSeconds = 0;

    protected int failedDAQs = 0;

    protected int lastThreadPriority = STARTING_THREAD_PRIORITY;
    protected volatile int threadPriorityToSet = lastThreadPriority;

    public static double[] writeBufferToArray(final DoubleCircularBuffer buffer) {
        final int size = buffer.size();
        final double[] array = new double[size];
        for (int i = 0; i < size; i++) {
            array[i] = buffer.get(i);
        }

        return array;
    }

    public OdometryThreadRunner(final ReentrantReadWriteLock signalQueueReadWriteLock) {
        this.thread = new Thread(this::run);
        this.thread.setDaemon(true);

        this.signalQueueReadWriteLock = signalQueueReadWriteLock;
    }

    /**
     * Starts the odometry thread.
     */
    public void start() {
        running = true;
        thread.start();
    }

    /**
     * Stops the odometry thread.
     */
    public void stop() {
        stop(0);
    }

    /**
     * Stops the odometry thread with a timeout.
     * @param timeoutMillis The time to wait in milliseconds
     */
    public void stop(final long timeoutMillis) {
        running = false;
        try {
            thread.join(timeoutMillis);
        } catch (final InterruptedException interruptedException) {
            Thread.currentThread().interrupt();
        }
    }

    /**
     * Gets a reference to the signal queue {@link ReentrantReadWriteLock} used to lock
     * read/write operations on any signal queue
     * @return the {@link ReentrantReadWriteLock}
     */
    public ReentrantReadWriteLock getSignalQueueReadWriteLock() {
        return signalQueueReadWriteLock;
    }

    /**
     * Sets the DAQ thread priority to a real time priority under the specified priority level
     * @param priority Priority level to set the DAQ thread to. This is a value between 0 and 99,
     *                 with 99 indicating higher priority and 0 indicating lower priority.
     */
    public void setThreadPriority(int priority) {
        threadPriorityToSet = priority;
    }

    public static class State implements StructSerializable {
        public static final StateStruct struct = new StateStruct();
        public boolean running;
        public int failedDAQs;
        public int statusCode;
        public int maxQueueSize;
        public double timestampSeconds;
        public double odometryPeriodSeconds;

        public static class StateStruct implements Struct<State> {
            @Override
            public Class<State> getTypeClass() {
                return State.class;
            }

            @Override
            public String getTypeName() {
                return "OdometryThreadRunner.State";
            }

            @Override
            public int getSize() {
                return kSizeBool + (kSizeInt32 * 4) + (kSizeDouble * 2);
            }

            @Override
            public String getSchema() {
                return "bool running;int32 failedDAQs;int32 statusCode;int32 maxQueueSize;double timestampSeconds;double odometryPeriodSeconds";
            }

            @Override
            public State unpack(final ByteBuffer bb) {
                final boolean running = bb.get() != 0;
                final int failedDAQs = bb.getInt();
                final int statusCode = bb.getInt();
                final int maxQueueSize = bb.getInt();
                final double timestamp = bb.getDouble();
                final double odometryPeriod = bb.getDouble();

                final State state = new State();
                state.running = running;
                state.failedDAQs = failedDAQs;
                state.statusCode = statusCode;
                state.maxQueueSize = maxQueueSize;
                state.timestampSeconds = timestamp;
                state.odometryPeriodSeconds = odometryPeriod;
                return state;
            }

            @Override
            public void pack(final ByteBuffer bb, final State value) {
                bb.put((byte)(value.running ? 1 : 0));
                bb.putInt(value.failedDAQs);
                bb.putInt(value.statusCode);
                bb.putInt(value.maxQueueSize);
                bb.putDouble(value.timestampSeconds);
                bb.putDouble(value.odometryPeriodSeconds);
            }
        }
    }

    /**
     * Gets the current state of the {@link OdometryThreadRunner}, describing failed DAQs,
     * actual (measured) odometry period, etc...
     * @return the internal {@link State}
     */
    public State getState() {
        try {
            signalQueueReadWriteLock.readLock().lock();
            return state;
        } finally {
            signalQueueReadWriteLock.readLock().unlock();
        }
    }

    public DoubleCircularBuffer makeTimestampBuffer() {
        final DoubleCircularBuffer buffer = new DoubleCircularBuffer(20);
        try {
            signalReadWriteLock.writeLock().lock();
            timestampBuffers.add(buffer);
        } finally {
            signalReadWriteLock.writeLock().unlock();
        }

        return buffer;
    }

    public <T extends Measure<?>> DoubleCircularBuffer registerSignal(
            final ParentDevice device,
            final StatusSignal<T> signal
    ) {
        final DoubleCircularBuffer buffer = new DoubleCircularBuffer(20);
        try {
            signalReadWriteLock.writeLock().lock();
            final String deviceNetwork = device.getNetwork();

            // Ensure that we cannot register devices on different networks
            if (network != null && !network.equals(deviceNetwork)) {
                throw new RuntimeException(String.format(
                        "Attempted to register signal from a device on a different network than devices already" +
                                "registered! Current: %s, New: %s! This is a bug!",
                        network,
                        deviceNetwork
                ));
            } else if (network == null || canBus == null) {
                network = deviceNetwork;
                canBus = new CANBus(deviceNetwork);
            }

            if (!canBus.isNetworkFD()) {
                DriverStation.reportWarning(String.format(
                        "Attempted to register signal from a non CAN-FD device ID: %d (%s)! This is a bug!",
                        device.getDeviceID(),
                        deviceNetwork
                ), true);
                // TODO: fix this, why does it do this?
//                throw new RuntimeException(String.format(
//                        "Attempted to register signal from a non CAN-FD device ID: %d (%s)! This is a bug!",
//                        device.getDeviceID(),
//                        deviceNetwork
//                ));
            }

            allSignals.add(signal);
            buffers.add(buffer);
        } finally {
            signalReadWriteLock.writeLock().unlock();
        }

        return buffer;
    }

    public void registerControlRequest(
            final ParentDevice device,
            final ControlRequest controlRequest,
            final Consumer<ControlRequest> applyControlReq
    ) {
        final long deviceHash = device.getDeviceHash();
        final String deviceNetwork = device.getNetwork();
        if (network != null && !network.equals(deviceNetwork)) {
            throw new RuntimeException(String.format(
                    "Attempted to register signal from a device on a different network than devices already" +
                            "registered! Current: %s, New: %s! This is a bug!",
                    network,
                    deviceNetwork
            ));
        } else if (network == null) {
            network = deviceNetwork;
        }

        if (outerAppliedControlRequests.containsKey(deviceHash)) {
            throw new RuntimeException(String.format(
                    "Attempted to register a ControlRequest for the same device" +
                            "ID: %d (%s) more than once!", device.getDeviceID(), deviceNetwork
            ));
        }

        outerAppliedControlRequests.put(deviceHash, controlRequest);
        try {
            controlReqReadWriteLock.writeLock().lock();

            innerAppliedControlRequests.put(deviceHash, controlRequest);
            controlReqAppliers.put(deviceHash, applyControlReq);
        } finally {
            controlReqReadWriteLock.writeLock().unlock();
        }
    }

    public void updateControlRequest(final ParentDevice device, final ControlRequest controlRequest) {
        final long deviceHash = device.getDeviceHash();
        if (outerAppliedControlRequests.get(deviceHash) == controlRequest) {
            return;
        }

        outerAppliedControlRequests.put(deviceHash, controlRequest);
        try {
            controlReqReadWriteLock.writeLock().lock();
            innerAppliedControlRequests.put(deviceHash, controlRequest);
        } finally {
            controlReqReadWriteLock.writeLock().unlock();
        }
    }

    private void run() {
        if (allSignals.isEmpty()) {
            DriverStation.reportError(
                    "Attempted to start OdometryThread without any registered signals!",
                    false
            );
            stop();

            return;
        }

        final BaseStatusSignal[] allSignalsArray = allSignals.toArray(BaseStatusSignal[]::new);
        BaseStatusSignal.setUpdateFrequencyForAll(UPDATE_FREQUENCY_HZ, allSignalsArray);
//        Threads.setCurrentThreadPriority(true, threadPriorityToSet);

        while (running) {
            final StatusCode statusCode;
            try {
                signalReadWriteLock.writeLock().lock();
                statusCode = BaseStatusSignal.waitForAll(
                        2.0 / UPDATE_FREQUENCY_HZ, allSignalsArray
                );


                if (!statusCode.isOK()) {
                    failedDAQs++;
                }
            } finally {
                signalReadWriteLock.writeLock().unlock();
            }

            try {
                signalQueueReadWriteLock.writeLock().lock();
                lastTimeSeconds = currentTimeSeconds;
                currentTimeSeconds = Utils.getCurrentTimeSeconds();

                // We don't care about the peaks, as they correspond to GC events,
                // and we want the period generally low passed
                averageLoopTimeSeconds = lowPass.calculate(
                        peakRemover.calculate(currentTimeSeconds - lastTimeSeconds)
                );

                state.running = running;
                state.failedDAQs = failedDAQs;
                state.odometryPeriodSeconds = averageLoopTimeSeconds;
                if (!statusCode.isOK()) {
                    state.statusCode = statusCode.value;
                }

                final int signalCount = allSignals.size();
                double totalLatencySeconds = 0;
                int maxQueueSize = 0;
                for (int i = 0; i < signalCount; i++) {
                    final DoubleCircularBuffer buffer = buffers.get(i);
                    final StatusSignal<?> signal = allSignals.get(i);
                    buffer.addFirst(signal.getValueAsDouble());

                    final int queueSize = buffer.size();
                    if (queueSize > maxQueueSize) {
                        maxQueueSize = queueSize;
                    }

                    totalLatencySeconds += signal.getTimestamp().getLatency();
                }

                final double realTimestampSeconds = RobotController.getFPGATime() / 1e6;
                final double signalTimestampSeconds = realTimestampSeconds - (totalLatencySeconds / signalCount);
                for (final DoubleCircularBuffer timestampBuffer : timestampBuffers) {
                    timestampBuffer.addFirst(signalTimestampSeconds);
                }

                state.timestampSeconds = signalTimestampSeconds;
                state.maxQueueSize = maxQueueSize;
            } finally {
                signalQueueReadWriteLock.writeLock().unlock();
            }

            try {
                controlReqReadWriteLock.readLock().lock();
                for (final Map.Entry<Long, Consumer<ControlRequest>>
                        controlReqApplierEntry : controlReqAppliers.entrySet()
                ) {
                    controlReqApplierEntry
                            .getValue()
                            .accept(innerAppliedControlRequests.get(controlReqApplierEntry.getKey()));
                }
            } finally {
                controlReqReadWriteLock.readLock().unlock();
            }

            // This is inherently synchronous, since lastThreadPriority is only written
            // here and threadPriorityToSet is only read here
            if (threadPriorityToSet != lastThreadPriority) {
//                Threads.setCurrentThreadPriority(true, threadPriorityToSet);
                lastThreadPriority = threadPriorityToSet;
            }
        }

        state.running = running;
    }
}