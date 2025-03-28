package frc.robot.subsystems.superstructure.distal;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.VoltageUnit;
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
    private static final double PositionToleranceRots = 0.0075;
    private static final double VelocityToleranceRotsPerSec = 0.01;

    private final IntakeArmIO intakeArmIO;
    private final IntakeArmIOInputsAutoLogged inputs;

    private final SysIdRoutine pivotVoltageSysIdRoutine;

    private Goal desiredGoal = Goal.STOW;
    private Goal currentGoal = desiredGoal;

    private final PositionSetpoint positionSetpoint;
    private final PositionSetpoint pivotLowerLimit;
    private final PositionSetpoint pivotUpperLimit;

    public final Trigger atSetpoint = new Trigger(this::atPivotPositionSetpoint);
    public final Trigger atPivotLowerLimit = new Trigger(this::atPivotLowerLimit);
    public final Trigger atPivotUpperLimit = new Trigger(this::atPivotUpperLimit);

    public static class PositionSetpoint {
        public double pivotPositionRots = 0.0;

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
            return PositionSetpoint.atSetpoint(
                    this.pivotPositionRots,
                    pivotPositionRots,
                    pivotVelocityRotsPerSec
            );
        }
    }

    public enum Goal {
        STOW(0),
        HP(0),
        ALGAE_GROUND(-0.349),
        UPPER_ALGAE(-0.35),
        LOWER_ALGAE(-0.324),
        PROCESSOR(-0.24),
        CLIMB(-0.348),
        NET(-0.15),
        L4(-0.17),
        L3(-0.174),
        L2(-0.127),
        L1(-0.052);

        private final double pivotPositionGoalRots;

        Goal(final double pivotPositionGoalRots) {
            this.pivotPositionGoalRots = pivotPositionGoalRots;
        }

        public double getPivotPositionGoalRots() {
            return pivotPositionGoalRots;
        }
    }

    public IntakeArm(final Constants.RobotMode mode, final HardwareConstants.IntakeArmConstants constants) {
        this.intakeArmIO = switch (mode) {
            case REAL -> new IntakeArmIOReal(constants);
            case SIM -> new IntakeArmIOSim(constants);
            case REPLAY, DISABLED -> new IntakeArmIO() {};
        };

        this.inputs = new IntakeArmIOInputsAutoLogged();

        this.pivotVoltageSysIdRoutine = makeVoltageSysIdRoutine(
                Volts.of(0.2).per(Second),
                Volts.of(2),
                Seconds.of(10),
                intakeArmIO::toPivotVoltage
        );

        this.positionSetpoint = new PositionSetpoint()
                .withPivotPositionRots(desiredGoal.getPivotPositionGoalRots());
        this.pivotLowerLimit = new PositionSetpoint().withPivotPositionRots(constants.pivotLowerLimitRots());
        this.pivotUpperLimit = new PositionSetpoint().withPivotPositionRots(constants.pivotUpperLimitRots());

        this.intakeArmIO.config();
        this.intakeArmIO.toPivotPosition(positionSetpoint.pivotPositionRots);
    }

    @Override
    public void periodic() {
        final double intakePeriodicUpdateStart = RobotController.getFPGATime();

        intakeArmIO.updateInputs(inputs);
        Logger.processInputs(LogKey, inputs);

        if (desiredGoal != currentGoal) {
            if (currentGoal == Goal.ALGAE_GROUND
                    || currentGoal == Goal.LOWER_ALGAE
                    || currentGoal == Goal.UPPER_ALGAE
                    || desiredGoal == Goal.PROCESSOR
            ) {
                intakeArmIO.setMotionMagicCruiseVelocity(0.45);
            } else {
                intakeArmIO.setMotionMagicCruiseVelocity(0);
            }

            positionSetpoint.pivotPositionRots = desiredGoal.getPivotPositionGoalRots();
            intakeArmIO.toPivotPosition(positionSetpoint.pivotPositionRots);

            this.currentGoal = desiredGoal;
        }

        Logger.recordOutput(LogKey + "/CurrentPivotGoal", currentGoal.toString());
        Logger.recordOutput(LogKey + "/DesiredPivotGoal", desiredGoal.toString());
        Logger.recordOutput(
                LogKey + "/PivotPositionSetpoint/PivotPositionRots",
                positionSetpoint.pivotPositionRots
        );
        Logger.recordOutput(LogKey + "/AtPositionSetpoint", atPivotPositionSetpoint());
        Logger.recordOutput(LogKey + "/AtLowerLimit", atPivotLowerLimit());
        Logger.recordOutput(LogKey + "/AtUpperLimit", atPivotUpperLimit());

        Logger.recordOutput(
                LogKey + "/PeriodicIOPeriodMs",
                LogUtils.microsecondsToMilliseconds(RobotController.getFPGATime() - intakePeriodicUpdateStart)
        );
    }

    public boolean atGoal(final Goal goal) {
        return PositionSetpoint.atSetpoint(
                goal.getPivotPositionGoalRots(),
                inputs.pivotPositionRots,
                inputs.pivotVelocityRotsPerSec
        );
    }

    private boolean atPivotPositionSetpoint() {
        return positionSetpoint.atSetpoint(inputs.pivotPositionRots, inputs.pivotVelocityRotsPerSec)
                && currentGoal == desiredGoal;
    }

    private boolean atPivotLowerLimit() {
        return inputs.pivotPositionRots <= pivotLowerLimit.pivotPositionRots;
    }

    private boolean atPivotUpperLimit() {
        return inputs.pivotPositionRots >= pivotUpperLimit.pivotPositionRots;
    }

    public Rotation2d getPivotPosition() {
        return Rotation2d.fromRotations(inputs.pivotPositionRots);
    }

    public void setGoal(final Goal goal) {
        this.desiredGoal = goal;
        Logger.recordOutput(LogKey + "/CurrentPivotGoal", currentGoal.toString());
        Logger.recordOutput(LogKey + "/DesiredPivotGoal", desiredGoal.toString());
    }

    public Command runPivotGoalCommand(final Goal goal) {
        return Commands.run(() -> setGoal(goal));
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
                sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse).until(atPivotLowerLimit),
                Commands.waitSeconds(1),
                sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward).until(atPivotUpperLimit),
                Commands.waitSeconds(1),
                sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse).until(atPivotLowerLimit),
                Commands.waitSeconds(1),
                sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward).until(atPivotUpperLimit)
        );
    }

    public Command pivotVoltageSysIdCommand() {
        return makePivotSysIdCommand(pivotVoltageSysIdRoutine);
    }
}
