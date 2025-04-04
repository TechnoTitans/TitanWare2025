package frc.robot.subsystems.superstructure.proximal;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
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
import frc.robot.constants.SimConstants;
import frc.robot.utils.logging.LogUtils;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.*;

public class ElevatorArm extends SubsystemBase {
    protected static final String LogKey = "ElevatorArm";
    private static final double PositionToleranceRots = 0.0065;
    private static final double VelocityToleranceRotsPerSec = 0.01;

    private final ElevatorArmIO elevatorArmIO;
    private final ElevatorArmIOInputsAutoLogged inputs;

    private final SysIdRoutine voltageSysIdRoutine;
    private final SysIdRoutine torqueCurrentSysIdRoutine;

    private Goal desiredGoal = Goal.STOW;
    private Goal currentGoal = desiredGoal;

    private final PositionSetpoint setpoint;
    private final PositionSetpoint pivotLowerSetpoint;
    private final PositionSetpoint pivotUpperSetpoint;

    public final Trigger atSetpoint = new Trigger(this::atPositionSetpoint);
    public final Trigger atPivotLowerLimit = new Trigger(this::atPivotLowerLimit);
    public final Trigger atPivotUpperLimit = new Trigger(this::atPivotUpperLimit);

    public static class PositionSetpoint {
        public double pivotPositionRots = 0;

        public PositionSetpoint withPivotPositionRots(final double pivotPositionRots) {
            this.pivotPositionRots = pivotPositionRots;
            return this;
        }

        public static boolean atSetpoint(
                final double setpointPivotPositionRots,
                final double pivotPositionRots,
                final double pivotVelocityRotsPerSec
        ) {
            return MathUtil.isNear(setpointPivotPositionRots, pivotPositionRots, PositionToleranceRots)
                    && MathUtil.isNear(0, pivotVelocityRotsPerSec, VelocityToleranceRotsPerSec);
        }

        public boolean atSetpoint(final double pivotPositionRots, final double pivotVelocityRotsPerSec) {
            return PositionSetpoint.atSetpoint(this.pivotPositionRots, pivotPositionRots, pivotVelocityRotsPerSec);
        }
    }

    public enum Goal {
        DYNAMIC(0),
        STOW(0.106),
        UPRIGHT(0.1844),
        HP(Units.degreesToRotations(33)),
        ALGAE_GROUND(0),
        PROCESSOR(Units.degreesToRotations(1)),
        UPPER_ALGAE(0.1211),
        LOWER_ALGAE(0.0904),
        L4(0.1525),
        L3(0.12842),
        L2(0.1),
        L1(0.0214),
        CLIMB(0.184),
        CLIMB_DOWN(-6.5);

        private final double pivotPositionGoalRots;
        Goal(final double pivotPositionGoalRots) {
            this.pivotPositionGoalRots = pivotPositionGoalRots;
        }

        public double getPivotPositionGoalRots() {
            return pivotPositionGoalRots;
        }
    }

    public ElevatorArm(final Constants.RobotMode mode, final HardwareConstants.ElevatorArmConstants constants) {
        this.elevatorArmIO = switch (mode) {
            case REAL -> new ElevatorArmIOReal(constants);
            case SIM -> new ElevatorArmIOSim(constants);
            case REPLAY, DISABLED -> new ElevatorArmIO() {};
        };

        this.inputs = new ElevatorArmIOInputsAutoLogged();

        this.voltageSysIdRoutine = makeVoltageSysIdRoutine(
                Volts.of(2).per(Second),
                Volts.of(3),
                Seconds.of(6)
        );
        this.torqueCurrentSysIdRoutine = makeTorqueCurrentSysIdRoutine(
                Amps.of(2).per(Second),
                Amps.of(8),
                Seconds.of(6)
        );

        this.setpoint = new PositionSetpoint().withPivotPositionRots(desiredGoal.getPivotPositionGoalRots());
        this.pivotLowerSetpoint = new PositionSetpoint().withPivotPositionRots(constants.lowerLimitRots());
        this.pivotUpperSetpoint = new PositionSetpoint().withPivotPositionRots(constants.upperLimitRots());

        this.elevatorArmIO.config();
        this.elevatorArmIO.toPivotPosition(setpoint.pivotPositionRots);
    }

    @Override
    public void periodic() {
        final double armPeriodicUpdateStart = RobotController.getFPGATime();

        elevatorArmIO.updateInputs(inputs);
        Logger.processInputs(LogKey, inputs);

        if (desiredGoal != currentGoal) {
            if (desiredGoal != Goal.DYNAMIC && desiredGoal != Goal.CLIMB_DOWN) {
                setpoint.pivotPositionRots = desiredGoal.getPivotPositionGoalRots();
                elevatorArmIO.toPivotPosition(setpoint.pivotPositionRots);
            }
            if (desiredGoal == Goal.CLIMB_DOWN) {
                setpoint.pivotPositionRots = 0;
                elevatorArmIO.toPivotVoltage(desiredGoal.pivotPositionGoalRots);
            }

            this.currentGoal = desiredGoal;
        }

        Logger.recordOutput(LogKey + "/CurrentGoal", currentGoal.toString());
        Logger.recordOutput(LogKey + "/DesiredGoal", desiredGoal.toString());
        Logger.recordOutput(LogKey + "/PositionSetpoint/PivotPositionRots", setpoint.pivotPositionRots);
        Logger.recordOutput(LogKey + "/AtPositionSetpoint", atPositionSetpoint());
        Logger.recordOutput(LogKey + "/AtPivotLowerLimit", atPivotLowerLimit());
        Logger.recordOutput(LogKey + "/AtPivotUpperLimit", atPivotUpperLimit());

        Logger.recordOutput(
                LogKey + "/PeriodicIOPeriodMs",
                LogUtils.microsecondsToMilliseconds(RobotController.getFPGATime() - armPeriodicUpdateStart)
        );
    }

    public boolean atGoal(final Goal goal) {
        return PositionSetpoint.atSetpoint(
                goal.getPivotPositionGoalRots(),
                inputs.pivotPositionRots,
                inputs.pivotVelocityRotsPerSec
        );
    }

    private boolean atPositionSetpoint() {
        return setpoint.atSetpoint(inputs.pivotPositionRots, inputs.pivotVelocityRotsPerSec)
                && currentGoal == desiredGoal;
    }

    private boolean atPivotLowerLimit() {
        return inputs.pivotPositionRots <= pivotLowerSetpoint.pivotPositionRots;
    }

    private boolean atPivotUpperLimit() {
        return inputs.pivotPositionRots >= pivotUpperSetpoint.pivotPositionRots;
    }

    public Rotation2d getPivotPosition() {
        return Rotation2d.fromRotations(inputs.pivotPositionRots);
    }

    public Rotation2d getPivotPositionRadsFromHorizontal() {
        return getPivotPosition().plus(SimConstants.ElevatorArm.ZEROED_POSITION_TO_HORIZONTAL);
    }

    public void setGoal(final Goal goal) {
        this.desiredGoal = goal;
        Logger.recordOutput(LogKey + "/CurrentGoal", currentGoal.toString());
        Logger.recordOutput(LogKey + "/DesiredGoal", desiredGoal.toString());
    }

    public Command runPositionCommand(final DoubleSupplier positionRots) {
        return run(() -> {
            this.desiredGoal = Goal.DYNAMIC;
            setpoint.pivotPositionRots = positionRots.getAsDouble();
            elevatorArmIO.toPivotPosition(setpoint.pivotPositionRots);
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
                        voltageMeasure -> elevatorArmIO.toPivotVoltage(
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
                        voltageMeasure -> elevatorArmIO.toPivotTorqueCurrent(
                                voltageMeasure.in(Volts)
                        ),
                        null,
                        this
                )
        );
    }

    private Command makeSysIdCommand(final SysIdRoutine sysIdRoutine) {
        return Commands.sequence(
                sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward).until(atPivotUpperLimit),
                Commands.waitSeconds(1),
                sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse).until(atPivotLowerLimit),
                Commands.waitSeconds(1),
                sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward).until(atPivotUpperLimit),
                Commands.waitSeconds(1),
                sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse).until(atPivotLowerLimit)
        );
    }

    public Command voltageSysIdCommand() {
        return makeSysIdCommand(voltageSysIdRoutine);
    }

    public Command torqueCurrentSysIdCommand() {
        return makeSysIdCommand(torqueCurrentSysIdRoutine);
    }
}
