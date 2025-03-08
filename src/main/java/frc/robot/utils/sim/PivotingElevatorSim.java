package frc.robot.utils.sim;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.NumericalIntegration;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;

import java.util.function.DoubleSupplier;

public class PivotingElevatorSim extends LinearSystemSim<N2, N1, N2> {
    // Gearbox for the elevator.
    private final DCMotor gearbox;

    // The min allowable height for the elevator.
    private final double minHeight;

    // The max allowable height for the elevator.
    private final double maxHeight;

    // Whether the simulator should simulate gravity.
    private final boolean simulateGravity;

    private DoubleSupplier angleRadsFromHorizontalSupplier;

    /**
     * Creates a simulated elevator mechanism.
     *
     * @param plant The linear system that represents the elevator. This system can be created with
     *     {@link edu.wpi.first.math.system.plant.LinearSystemId#createElevatorSystem(DCMotor, double,
     *     double, double)}.
     * @param gearbox The type of and number of motors in the elevator gearbox.
     * @param minHeightMeters The min allowable height of the elevator.
     * @param maxHeightMeters The max allowable height of the elevator.
     * @param simulateGravity Whether gravity should be simulated or not.
     * @param startingHeightMeters The starting height of the elevator.
     * @param measurementStdDevs The standard deviations of the measurements. Can be omitted if no
     *     noise is desired. If present must have 1 element for position.
     */
    @SuppressWarnings("this-escape")
    public PivotingElevatorSim(
            final LinearSystem<N2, N1, N2> plant,
            final DCMotor gearbox,
            final double minHeightMeters,
            final double maxHeightMeters,
            final boolean simulateGravity,
            final double startingHeightMeters,
            final DoubleSupplier startingAngleRads,
            final double... measurementStdDevs) {
        super(plant, measurementStdDevs);
        this.gearbox = gearbox;
        this.minHeight = minHeightMeters;
        this.maxHeight = maxHeightMeters;
        this.simulateGravity = simulateGravity;
        this.angleRadsFromHorizontalSupplier = startingAngleRads;

        setState(startingHeightMeters, 0);
    }

    /**
     * Creates a simulated elevator mechanism.
     *
     * @param kV The velocity gain.
     * @param kA The acceleration gain.
     * @param gearbox The type of and number of motors in the elevator gearbox.
     * @param minHeightMeters The min allowable height of the elevator.
     * @param maxHeightMeters The max allowable height of the elevator.
     * @param simulateGravity Whether gravity should be simulated or not.
     * @param startingHeightMeters The starting height of the elevator.
     * @param measurementStdDevs The standard deviations of the measurements. Can be omitted if no
     *     noise is desired. If present must have 1 element for position.
     */
    public PivotingElevatorSim(
            final double kV,
            final double kA,
            final DCMotor gearbox,
            final double minHeightMeters,
            final double maxHeightMeters,
            final boolean simulateGravity,
            final double startingHeightMeters,
            final DoubleSupplier startingAngleRadsSupplier,
            final double... measurementStdDevs) {
        this(
                LinearSystemId.identifyPositionSystem(kV, kA),
                gearbox,
                minHeightMeters,
                maxHeightMeters,
                simulateGravity,
                startingHeightMeters,
                startingAngleRadsSupplier,
                measurementStdDevs);
    }

    /**
     * Creates a simulated elevator mechanism.
     *
     * @param gearbox The type of and number of motors in the elevator gearbox.
     * @param gearing The gearing of the elevator (numbers greater than 1 represent reductions).
     * @param carriageMassKg The mass of the elevator carriage.
     * @param drumRadiusMeters The radius of the drum that the elevator spool is wrapped around.
     * @param minHeightMeters The min allowable height of the elevator.
     * @param maxHeightMeters The max allowable height of the elevator.
     * @param simulateGravity Whether gravity should be simulated or not.
     * @param startingHeightMeters The starting height of the elevator.
     * @param measurementStdDevs The standard deviations of the measurements. Can be omitted if no
     *     noise is desired. If present must have 1 element for position.
     */
    public PivotingElevatorSim(
            final DCMotor gearbox,
            final double gearing,
            final double carriageMassKg,
            final double drumRadiusMeters,
            final double minHeightMeters,
            final double maxHeightMeters,
            final boolean simulateGravity,
            final double startingHeightMeters,
            final DoubleSupplier startingAngleRadsSupplier,
            final double... measurementStdDevs) {
        this(
                LinearSystemId.createElevatorSystem(gearbox, carriageMassKg, drumRadiusMeters, gearing),
                gearbox,
                minHeightMeters,
                maxHeightMeters,
                simulateGravity,
                startingHeightMeters,
                startingAngleRadsSupplier,
                measurementStdDevs);
    }

    /**
     * Sets the elevator's state. The new position will be limited between the minimum and maximum
     * allowed heights.
     *
     * @param positionMeters The new position in meters.
     * @param velocityMetersPerSecond New velocity in meters per second.
     */
    public final void setState(double positionMeters, double velocityMetersPerSecond) {
        setState(
                VecBuilder.fill(
                        MathUtil.clamp(positionMeters, minHeight, maxHeight), velocityMetersPerSecond));
    }

    /**
     * Returns whether the elevator would hit the lower limit.
     *
     * @param elevatorHeightMeters The elevator height.
     * @return Whether the elevator would hit the lower limit.
     */
    public boolean wouldHitLowerLimit(double elevatorHeightMeters) {
        return elevatorHeightMeters <= this.minHeight;
    }

    /**
     * Returns whether the elevator would hit the upper limit.
     *
     * @param elevatorHeightMeters The elevator height.
     * @return Whether the elevator would hit the upper limit.
     */
    public boolean wouldHitUpperLimit(double elevatorHeightMeters) {
        return elevatorHeightMeters >= this.maxHeight;
    }

    /**
     * Returns whether the elevator has hit the lower limit.
     *
     * @return Whether the elevator has hit the lower limit.
     */
    public boolean hasHitLowerLimit() {
        return wouldHitLowerLimit(getPositionMeters());
    }

    /**
     * Returns whether the elevator has hit the upper limit.
     *
     * @return Whether the elevator has hit the upper limit.
     */
    public boolean hasHitUpperLimit() {
        return wouldHitUpperLimit(getPositionMeters());
    }

    /**
     * Returns the position of the elevator.
     *
     * @return The position of the elevator.
     */
    public double getPositionMeters() {
        return getOutput(0);
    }

    /**
     * Returns the velocity of the elevator.
     *
     * @return The velocity of the elevator.
     */
    public double getVelocityMetersPerSecond() {
        return getOutput(1);
    }

    /**
     * Returns the elevator current draw.
     *
     * @return The elevator current draw.
     */
    public double getCurrentDrawAmps() {
        // I = V / R - omega / (Kv * R)
        // Reductions are greater than 1, so a reduction of 10:1 would mean the motor is
        // spinning 10x faster than the output
        // v = r w, so w = v/r
        double kA = 1 / m_plant.getB().get(1, 0);
        double kV = -m_plant.getA().get(1, 1) * kA;
        double motorVelocityRadPerSec = m_x.get(1, 0) * kV * gearbox.KvRadPerSecPerVolt;
        var appliedVoltage = m_u.get(0, 0);
        return gearbox.getCurrent(motorVelocityRadPerSec, appliedVoltage)
                * Math.signum(appliedVoltage);
    }

    /**
     * Sets the input voltage for the elevator.
     *
     * @param volts The input voltage.
     */
    public void setInputVoltage(double volts) {
        setInput(volts);
        clampInput(RobotController.getBatteryVoltage());
    }

    /**
     * Updates the state of the elevator.
     *
     * @param currentXhat The current state estimate.
     * @param u The system inputs (voltage).
     * @param dtSeconds The time difference between controller updates.
     */
    @Override
    protected Matrix<N2, N1> updateX(Matrix<N2, N1> currentXhat, Matrix<N1, N1> u, double dtSeconds) {
        // Calculate updated x-hat from Runge-Kutta.
        var updatedXhat =
                NumericalIntegration.rkdp(
                        (x, _u) -> {
                            Matrix<N2, N1> xdot = m_plant.getA().times(x).plus(m_plant.getB().times(_u));
                            if (simulateGravity) {
                                xdot = xdot.plus(VecBuilder.fill(
                                        0,
                                        -9.8 * Math.sin(angleRadsFromHorizontalSupplier.getAsDouble())
                                ));
                            }
                            return xdot;
                        },
                        currentXhat,
                        u,
                        dtSeconds);

        // We check for collisions after updating x-hat.
        if (wouldHitLowerLimit(updatedXhat.get(0, 0))) {
            return VecBuilder.fill(minHeight, 0);
        }
        if (wouldHitUpperLimit(updatedXhat.get(0, 0))) {
            return VecBuilder.fill(maxHeight, 0);
        }
        return updatedXhat;
    }
}
