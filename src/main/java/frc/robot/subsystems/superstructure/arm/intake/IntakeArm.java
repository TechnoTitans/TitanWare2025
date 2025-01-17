package frc.robot.subsystems.superstructure.arm.intake;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.MathUtil;
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

import java.util.function.Consumer;

import static edu.wpi.first.units.Units.*;

public class IntakeArm extends SubsystemBase {
    protected static final String LogKey = "IntakeArm";
    private static final double PositionToleranceRots = 0.005;
    private static final double VelocityToleranceRotsPerSec = 0.01;

    private final IntakeArmIO intakeIO;
    private final IntakeArmIOInputsAutoLogged inputs;

    private final SysIdRoutine pivotVoltageSysIdRoutine;
    private final SysIdRoutine pivotTorqueCurrentSysIdRoutine;

    private IntakeArm.PivotGoal desiredPivotGoal = IntakeArm.PivotGoal.STOW;
    private IntakeArm.PivotGoal currentPivotGoal = desiredPivotGoal;

    private final IntakeArm.PivotPositionSetpoint pivotPositionSetpoint;
    private final IntakeArm.PivotPositionSetpoint pivotLowerLimit;
    private final IntakeArm.PivotPositionSetpoint pivotUpperLimit;

    public final Trigger atPivotPositionSetpoint = new Trigger(this::atPivotPositionSetpoint);
    public final Trigger atPivotLowerLimit = new Trigger(this::atPivotLowerLimit);
    public final Trigger atPivotUpperLimit = new Trigger(this::atPivotUpperLimit);

    public static class PivotPositionSetpoint {
        public double pivotPositionRots = 0.0;

        public IntakeArm.PivotPositionSetpoint withPivotPositionRots(final double pivotPositionRots) {
            this.pivotPositionRots = pivotPositionRots;
            return this;
        }

        public boolean atSetpoint(final double pivotPositionRots, final double pivotVelocityRotsPerSec) {
            return MathUtil.isNear(this.pivotPositionRots, pivotPositionRots, PositionToleranceRots)
                    && MathUtil.isNear(0, pivotVelocityRotsPerSec, VelocityToleranceRotsPerSec);
        }
    }

    public enum PivotGoal {
        STOW(0),
        HP(2),
        ALGAE_GROUND(2),
        ALGAE_REEF(2),
        NET(2),
        L1(1),
        L2(2),
        L3(2),
        L4(4);

        private final double pivotPositionGoalRots;

        PivotGoal(final double pivotPositionGoalRots) {
            this.pivotPositionGoalRots = pivotPositionGoalRots;
        }

        public double getPivotPositionGoalRots() {
            return pivotPositionGoalRots;
        }
    }

    public IntakeArm(final Constants.RobotMode mode, final HardwareConstants.IntakeArmConstants constants) {
        this.intakeIO = switch (mode) {
            case REAL -> new IntakeArmIOReal(constants);
            case SIM -> new IntakeArmIOSim(constants);
            case REPLAY, DISABLED -> new IntakeArmIO() {};
        };

        this.inputs = new IntakeArmIOInputsAutoLogged();

        this.pivotVoltageSysIdRoutine = makeVoltageSysIdRoutine(
                Volts.of(2).per(Second),
                Volts.of(10),
                Seconds.of(10),
                intakeIO::toPivotVoltage
        );
        this.pivotTorqueCurrentSysIdRoutine = makeTorqueCurrentSysIdRoutine(
                Amps.of(4).per(Second),
                Amp.of(40),
                Seconds.of(10),
                intakeIO::toPivotTorqueCurrent
        );

        this.pivotPositionSetpoint = new IntakeArm.PivotPositionSetpoint();
        this.pivotLowerLimit = new IntakeArm.PivotPositionSetpoint().withPivotPositionRots(constants.pivotLowerLimitRots());
        this.pivotUpperLimit = new IntakeArm.PivotPositionSetpoint().withPivotPositionRots(constants.pivotUpperLimitRots());

        this.intakeIO.config();
    }

    @Override
    public void periodic() {
        final double intakePeriodicUpdateStart = RobotController.getFPGATime();

        intakeIO.updateInputs(inputs);
        Logger.processInputs(LogKey, inputs);

        if (currentPivotGoal != desiredPivotGoal) {
            pivotPositionSetpoint.pivotPositionRots = desiredPivotGoal.getPivotPositionGoalRots();
            this.currentPivotGoal = desiredPivotGoal;
        }

        Logger.recordOutput(LogKey + "/CurrentPivotGoal", currentPivotGoal.toString());
        Logger.recordOutput(LogKey + "/DesiredPivotGoal", desiredPivotGoal.toString());
        Logger.recordOutput(
                LogKey + "/PivotPositionSetpoint/PivotPositionRots",
                pivotPositionSetpoint.pivotPositionRots
        );
        Logger.recordOutput(LogKey + "/AtPositionSetpoint", atPivotPositionSetpoint());
        Logger.recordOutput(LogKey + "/AtLowerLimit", atPivotLowerLimit());
        Logger.recordOutput(LogKey + "/AtUpperLimit", atPivotUpperLimit());

        Logger.recordOutput(
                LogKey + "/PeriodicIOPeriodMs",
                LogUtils.microsecondsToMilliseconds(RobotController.getFPGATime() - intakePeriodicUpdateStart)
        );
    }

    private boolean atPivotPositionSetpoint() {
        return pivotPositionSetpoint.atSetpoint(inputs.pivotPositionRots, inputs.pivotVelocityRotsPerSec)
                && currentPivotGoal == desiredPivotGoal;
    }

    private boolean atPivotLowerLimit() {
        return inputs.pivotPositionRots <= pivotLowerLimit.pivotPositionRots;
    }

    private boolean atPivotUpperLimit() {
        return inputs.pivotPositionRots >= pivotUpperLimit.pivotPositionRots;
    }

    public void setPivotGoal(final IntakeArm.PivotGoal goal) {
        this.desiredPivotGoal = goal;
        Logger.recordOutput(LogKey + "/CurrentPivotGoal", currentPivotGoal.toString());
        Logger.recordOutput(LogKey + "/DesiredPivotGoal", desiredPivotGoal.toString());
    }

    private SysIdRoutine makeVoltageSysIdRoutine(
            final Velocity<VoltageUnit> voltageRampRate,
            final Voltage stepVoltage,
            final Time timeout,
            final Consumer<Double> voltageConsumer
    ) {
        return new SysIdRoutine(
                new SysIdRoutine.Config(
                        voltageRampRate,
                        stepVoltage,
                        timeout,
                        state -> SignalLogger.writeString(String.format("%s-state", LogKey), state.toString())
                ),
                new SysIdRoutine.Mechanism(
                        voltageMeasure -> voltageConsumer.accept(voltageMeasure.in(Volts)),
                        null,
                        this
                )
        );
    }

    private SysIdRoutine makeTorqueCurrentSysIdRoutine(
            final Velocity<CurrentUnit> currentRampRate,
            final Current stepCurrent,
            final Time timeout,
            final Consumer<Double> torqueCurrentConsumer
    ) {
        return new SysIdRoutine(
                new SysIdRoutine.Config(
                        Volts.per(Second).of(currentRampRate.baseUnitMagnitude()),
                        Volts.of(stepCurrent.baseUnitMagnitude()),
                        timeout,
                        state -> SignalLogger.writeString(String.format("%s-state", LogKey), state.toString())
                ),
                new SysIdRoutine.Mechanism(
                        voltageMeasure -> torqueCurrentConsumer.accept(voltageMeasure.in(Volts)),
                        null,
                        this
                )
        );
    }

    private Command makeRollerSysIdCommand(final SysIdRoutine sysIdRoutine) {
        return Commands.sequence(
                sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward),
                Commands.waitSeconds(1),
                sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse),
                Commands.waitSeconds(1),
                sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward),
                Commands.waitSeconds(1),
                sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse)
        );
    }

    private Command makePivotSysIdCommand(final SysIdRoutine sysIdRoutine) {
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

    public Command pivotVoltageSysIdCommand() {
        return makePivotSysIdCommand(pivotVoltageSysIdRoutine);
    }
    public Command pivotTorqueCurrentSysIdCommand() {
        return makePivotSysIdCommand(pivotTorqueCurrentSysIdRoutine);
    }
}
