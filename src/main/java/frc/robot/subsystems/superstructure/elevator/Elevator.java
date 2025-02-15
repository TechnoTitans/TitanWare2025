package frc.robot.subsystems.superstructure.elevator;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.CurrentUnit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.Constants;
import frc.robot.constants.HardwareConstants;
import frc.robot.utils.logging.LogUtils;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.*;

public class Elevator extends SubsystemBase {
    protected static final String LogKey = "Elevator";
    private static final double PositionToleranceRots = 0.05;
    private static final double VelocityToleranceRotsPerSec = 0.05;

    private final HardwareConstants.ElevatorConstants constants;
    private final double drumCircumferenceMeters;

    private final ElevatorIO elevatorIO;
    private final ElevatorIOInputsAutoLogged inputs;

    private final SysIdRoutine voltageSysIdRoutine;
    private final SysIdRoutine torqueCurrentSysIdRoutine;

    private Goal desiredGoal = Goal.IDLE;
    private Goal currentGoal = desiredGoal;

    private final PositionSetpoint setpoint;
    private final PositionSetpoint elevatorLowerLimit;
    private final PositionSetpoint elevatorUpperLimit;

    public final Trigger atSetpoint = new Trigger(this::atPositionSetpoint);
    public final Trigger atLowerLimit = new Trigger(this::atLowerLimit);
    public final Trigger atUpperLimit = new Trigger(this::atUpperLimit);
    public final Trigger isRetracted = new Trigger(this::isRetracted);

    public static class PositionSetpoint {
        public double elevatorPositionRots = 0.0;

        public PositionSetpoint withElevatorPositionRots(final double elevatorPositionRots) {
            this.elevatorPositionRots = elevatorPositionRots;
            return this;
        }

        public boolean atSetpoint(final double elevatorPositionRots, final double elevatorVelocityRotsPerSec) {
            return MathUtil.isNear(this.elevatorPositionRots, elevatorPositionRots, PositionToleranceRots)
                    && MathUtil.isNear(0, elevatorVelocityRotsPerSec, VelocityToleranceRotsPerSec);
        }
    }

    public enum Goal {
        DYNAMIC(0),
        IDLE(0),
        HP(0.02),
        ALGAE_GROUND(0),
        L1(0.028),
        L2(0.150),
        LOWER_ALGAE(0.16),
        L3(0.458),
        UPPER_ALGAE(0.5),
        L4(0.976),
        NET(1);

        private final double positionGoalMeters;
        Goal(final double positionGoalMeters) {
            this.positionGoalMeters = positionGoalMeters;
        }

        public double getPositionGoalRots(final HardwareConstants.ElevatorConstants constants) {
            return positionGoalMeters / (constants.spoolDiameterMeters() * Math.PI);
        }

        public double getPositionGoalMeters() {
            return positionGoalMeters;
        }
    }

    public Elevator(final Constants.RobotMode mode, final HardwareConstants.ElevatorConstants constants) {
        this.constants = constants;
        this.drumCircumferenceMeters = constants.spoolDiameterMeters() * Math.PI;

        this.elevatorIO = switch (mode) {
            case REAL -> new ElevatorIOReal(constants);
            case SIM -> new ElevatorIOSim(constants);
            case REPLAY, DISABLED -> new ElevatorIO() {};
        };

        this.inputs = new ElevatorIOInputsAutoLogged();

        this.voltageSysIdRoutine = makeVoltageSysIdRoutine(
                Volts.of(0.5).per(Second),
                Volts.of(4),
                Seconds.of(10)
        );
        this.torqueCurrentSysIdRoutine = makeTorqueCurrentSysIdRoutine(
                Amps.of(4).per(Second),
                Amp.of(40),
                Seconds.of(10)
        );

        this.setpoint = new PositionSetpoint()
                .withElevatorPositionRots(desiredGoal.getPositionGoalRots(constants));
        this.elevatorLowerLimit = new PositionSetpoint().withElevatorPositionRots(constants.lowerLimitRots());
        this.elevatorUpperLimit = new PositionSetpoint().withElevatorPositionRots(constants.upperLimitRots());

        this.elevatorIO.config();
        this.home();
        this.elevatorIO.toPosition(setpoint.elevatorPositionRots);
    }

    @Override
    public void periodic() {
        final double elevatorPeriodicUpdateStart = RobotController.getFPGATime();

        elevatorIO.updateInputs(inputs);
        Logger.processInputs(LogKey, inputs);

        if (desiredGoal != currentGoal) {
            if (desiredGoal != Goal.DYNAMIC) {
                setpoint.elevatorPositionRots = desiredGoal.getPositionGoalRots(constants);
                elevatorIO.toPosition(setpoint.elevatorPositionRots);
            }

            this.currentGoal = desiredGoal;
        }

        Logger.recordOutput(LogKey + "/CurrentGoal", currentGoal.toString());
        Logger.recordOutput(LogKey + "/DesiredGoal", desiredGoal.toString());
        Logger.recordOutput(LogKey + "/PositionSetpoint/PositionRots", setpoint.elevatorPositionRots);
        Logger.recordOutput(LogKey + "/AtPositionSetpoint", atPositionSetpoint());
        Logger.recordOutput(LogKey + "/AtLowerLimit", atLowerLimit());
        Logger.recordOutput(LogKey + "/AtUpperLimit", atUpperLimit());
        Logger.recordOutput(LogKey + "/ExtensionMeters", getExtensionMeters());

        Logger.recordOutput(
                LogKey + "/PeriodicIOPeriodMs",
                LogUtils.microsecondsToMilliseconds(RobotController.getFPGATime() - elevatorPeriodicUpdateStart)
        );
    }

    private boolean atPositionSetpoint() {
        return setpoint.atSetpoint(inputs.masterPositionRots, inputs.masterVelocityRotsPerSec)
                && currentGoal == desiredGoal;
    }

    private boolean atLowerLimit() {
        return inputs.masterPositionRots <= elevatorLowerLimit.elevatorPositionRots;
    }

    private boolean atUpperLimit() {
        return inputs.masterPositionRots >= elevatorUpperLimit.elevatorPositionRots;
    }

    private boolean isRetracted() {
        return inputs.canRangeIsDetected;
    }

    public double getExtensionMeters() {
        return inputs.canRangeDistanceMeters;
    }


    //get distance from canrange minus thickness of elevator bottom plates
    public void home() {
        this.elevatorIO.setPosition(0);
//        this.elevatorIO.setPosition(
//                (inputs.canRangeDistanceMeters - Units.inchesToMeters(0.50)) / drumCircumferenceMeters
//        );
    }

    public Command homeCommand() {
        return runOnce(this::home);
    }

    public void setGoal(final Goal goal) {
        this.desiredGoal = goal;
        Logger.recordOutput(LogKey + "/CurrentGoal", currentGoal.toString());
        Logger.recordOutput(LogKey + "/DesiredGoal", desiredGoal.toString());
    }

    public Command toPositionMetersCommand(final DoubleSupplier positionMeters) {
        return run(() -> {
            this.desiredGoal = Goal.DYNAMIC;
            setpoint.elevatorPositionRots = positionMeters.getAsDouble() / drumCircumferenceMeters;
            elevatorIO.toPosition(setpoint.elevatorPositionRots);
        });
    }

    private SysIdRoutine makeVoltageSysIdRoutine(
            final Velocity<VoltageUnit> voltageRampRate,
            final Voltage stepVoltage,
            final Time timeout
    ) {
        return new SysIdRoutine(
                new SysIdRoutine.Config(
                        voltageRampRate,
                        stepVoltage,
                        timeout,
                        state -> SignalLogger.writeString(String.format("%s-state", LogKey), state.toString())
                ),
                new SysIdRoutine.Mechanism(
                        voltageMeasure -> elevatorIO.toVoltage(
                                voltageMeasure.in(Volts)
                        ),
                        null,
                        this
                )
        );
    }

    private SysIdRoutine makeTorqueCurrentSysIdRoutine(
            final Velocity<CurrentUnit> currentRampRate,
            final Current stepCurrent,
            final Time timeout
    ) {
        return new SysIdRoutine(
                new SysIdRoutine.Config(
                        Volts.per(Second).of(currentRampRate.baseUnitMagnitude()),
                        Volts.of(stepCurrent.baseUnitMagnitude()),
                        timeout,
                        state -> SignalLogger.writeString(String.format("%s-state", LogKey), state.toString())
                ),
                new SysIdRoutine.Mechanism(
                        voltageMeasure -> elevatorIO.toTorqueCurrent(
                                voltageMeasure.in(Volts)
                        ),
                        null,
                        this
                )
        );
    }

    private Command makeSysIdCommand(final SysIdRoutine sysIdRoutine) {
        return Commands.sequence(
                sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward).until(atUpperLimit),
                Commands.waitSeconds(0.1),
                sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse).until(atLowerLimit),
                Commands.waitSeconds(0.1),
                sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward).until(atUpperLimit),
                Commands.waitSeconds(0.1),
                sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse).until(atLowerLimit)
        );
    }

    public Command voltageSysIdCommand() {
        return makeSysIdCommand(voltageSysIdRoutine);
    }

    public Command torqueCurrentSysIdCommand() {
        return makeSysIdCommand(torqueCurrentSysIdRoutine);
    }
}
