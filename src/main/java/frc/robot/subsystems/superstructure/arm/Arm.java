package frc.robot.subsystems.superstructure.arm;

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

import static edu.wpi.first.units.Units.*;

public class Arm extends SubsystemBase {
    protected static final String LogKey = "Arm";
    private static final double PositionToleranceRots = 0.005;
    private static final double VelocityToleranceRotsPerSec = 0.01;

    private final ArmIO armIO;
    private final ArmIOInputsAutoLogged inputs;

    private final SysIdRoutine voltageSysIdRoutine;
    private final SysIdRoutine torqueCurrentSysIdRoutine;

    private Goal desiredGoal = Goal.STOW;
    private Goal currentGoal = desiredGoal;

    private final PositionSetpoint setpoint;
    private final PositionSetpoint pivotLowerSetpoint;
    private final PositionSetpoint pivotUpperSetpoint;

    private final Trigger atPivotSetpoint = new Trigger(this::atPositionSetpoint);
    private final Trigger atPivotLowerLimit = new Trigger(this::atPivotLowerLimit);
    private final Trigger atPivotUpperLimit = new Trigger(this::atPivotUpperLimit);

    public static class PositionSetpoint {
        public double pivotPositionRots = 0;

        public PositionSetpoint withPivotPositionRots(final double pivotPositionRots) {
            this.pivotPositionRots = pivotPositionRots;
            return this;
        }

        public boolean atSetpoint(final double pivotPositionRots, final double pivotVelocityRotsPerSec) {
            return MathUtil.isNear(this.pivotPositionRots, pivotPositionRots, PositionToleranceRots)
                    && MathUtil.isNear(0, pivotVelocityRotsPerSec, VelocityToleranceRotsPerSec);
        }
    }

    public enum Goal {
        STOW(Units.degreesToRotations(10)),
        UPRIGHT(Units.degreesToRotations(90)),
        CLIMB(Units.degreesToRotations(20));

        private final double pivotPositionGoalRots;
        Goal(final double pivotPositionGoalRots) {
            this.pivotPositionGoalRots = pivotPositionGoalRots;
        }

        public double getPivotPositionGoalRots() {
            return pivotPositionGoalRots;
        }
    }

    public Arm(final Constants.RobotMode mode, final HardwareConstants.ArmConstants constants) {
        this.armIO = switch (mode) {
            case REAL -> new ArmIOReal(constants);
            case SIM -> new ArmIOSim(constants);
            case REPLAY, DISABLED -> new ArmIO() {};
        };

        this.inputs = new ArmIOInputsAutoLogged();

        this.voltageSysIdRoutine = makeVoltageSysIdRoutine(
                Volts.of(4).per(Second),
                Volts.of(8),
                Seconds.of(6)
        );
        this.torqueCurrentSysIdRoutine = makeTorqueCurrentSysIdRoutine(
                Amps.of(2).per(Second),
                Amps.of(8),
                Seconds.of(6)
        );

        this.setpoint = new PositionSetpoint();
        this.pivotLowerSetpoint = new PositionSetpoint().withPivotPositionRots(constants.lowerLimitRots());
        this.pivotUpperSetpoint = new PositionSetpoint().withPivotPositionRots(constants.upperLimitRots());

        this.armIO.config();
    }

    @Override
    public void periodic() {
        final double armPeriodicUpdateStart = RobotController.getFPGATime();

        armIO.updateInputs(inputs);
        Logger.processInputs(LogKey, inputs);

        if (currentGoal != desiredGoal) {
            setpoint.pivotPositionRots = desiredGoal.getPivotPositionGoalRots();
            armIO.toPivotPosition(setpoint.pivotPositionRots);

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

    public void setGoal(final Goal goal) {
        this.desiredGoal = goal;
        Logger.recordOutput(LogKey + "/CurrentGoal", currentGoal.toString());
        Logger.recordOutput(LogKey + "/DesiredGoal", desiredGoal.toString());
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
                        voltageMeasure -> armIO.toPivotVoltage(
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
                        voltageMeasure -> armIO.toPivotTorqueCurrent(
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
                Commands.waitSeconds(4),
                sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse).until(atPivotLowerLimit),
                Commands.waitSeconds(4),
                sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward).until(atPivotUpperLimit),
                Commands.waitSeconds(4),
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
