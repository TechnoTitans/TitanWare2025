package frc.robot.subsystems.superstructure.ground;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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
import static edu.wpi.first.units.Units.Volts;

public class GroundIntakeArm extends SubsystemBase {
    protected static final String LogKey = "GroundIntakeArm";
    private static final double PositionToleranceRots = 0.031;
    private static final double VelocityToleranceRotsPerSec = 0.26;

    private final HardwareConstants.GroundIntakeArmConstants constants;
    private final Translation2d boundingBoxSize;

    private final GroundIntakeArmIO groundIntakeArmIO;
    private final GroundIntakeArmIOInputsAutoLogged inputs;

    private final SysIdRoutine pivotVoltageSysIdRoutine;

    private GroundIntakeArm.Goal desiredGoal = GroundIntakeArm.Goal.STOW;
    private GroundIntakeArm.Goal currentGoal = desiredGoal;

    private final GroundIntakeArm.PositionSetpoint positionSetpoint;
    private final GroundIntakeArm.PositionSetpoint pivotLowerLimit;
    private final GroundIntakeArm.PositionSetpoint pivotUpperLimit;

    public final Trigger atSetpoint = new Trigger(this::atPivotPositionSetpoint);
    public final Trigger atPivotLowerLimit = new Trigger(this::atPivotLowerLimit);
    public final Trigger atPivotUpperLimit = new Trigger(this::atPivotUpperLimit);

    public static class PositionSetpoint {
        public double pivotPositionRots = 0.0;

        public GroundIntakeArm.PositionSetpoint withPivotPositionRots(final double pivotPositionRots) {
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
            return GroundIntakeArm.PositionSetpoint.atSetpoint(
                    this.pivotPositionRots,
                    pivotPositionRots,
                    pivotVelocityRotsPerSec
            );
        }
    }

    public enum Goal {
        ZERO(0),
        STOW(-0.0243),
        HANDOFF(-0.0243),
        INTAKE(-0.324);

        private final double pivotPositionGoalRots;

        Goal(final double pivotPositionGoalRots) {
            this.pivotPositionGoalRots = pivotPositionGoalRots;
        }

        public double getPivotPositionGoalRots() {
            return pivotPositionGoalRots;
        }
    }

    public GroundIntakeArm(final Constants.RobotMode mode, final HardwareConstants.GroundIntakeArmConstants constants) {
        this.constants = constants;
        this.boundingBoxSize = new Translation2d(constants.lengthMeters(), constants.heightMeters());
        this.groundIntakeArmIO = switch (mode) {
            case REAL -> new GroundIntakeArmIOReal(constants);
            case SIM -> new GroundIntakeArmIOSim(constants);
            case REPLAY, DISABLED -> new GroundIntakeArmIO() {};
        };

        this.inputs = new GroundIntakeArmIOInputsAutoLogged();

        this.pivotVoltageSysIdRoutine = makeVoltageSysIdRoutine(
                Volts.of(0.2).per(Second),
                Volts.of(2),
                Seconds.of(10),
                groundIntakeArmIO::toPivotVoltage
        );

        this.positionSetpoint = new GroundIntakeArm.PositionSetpoint()
                .withPivotPositionRots(desiredGoal.getPivotPositionGoalRots());
        this.pivotLowerLimit = new GroundIntakeArm.PositionSetpoint()
                .withPivotPositionRots(constants.pivotLowerLimitRots());
        this.pivotUpperLimit = new GroundIntakeArm.PositionSetpoint()
                .withPivotPositionRots(constants.pivotUpperLimitRots());

        this.groundIntakeArmIO.config();
        this.groundIntakeArmIO.toPivotPosition(positionSetpoint.pivotPositionRots);
    }

    @Override
    public void periodic() {
        final double intakePeriodicUpdateStart = RobotController.getFPGATime();

        groundIntakeArmIO.updateInputs(inputs);
        Logger.processInputs(LogKey, inputs);

        if (desiredGoal != currentGoal) {
            positionSetpoint.pivotPositionRots = desiredGoal.getPivotPositionGoalRots();
            groundIntakeArmIO.toPivotPosition(positionSetpoint.pivotPositionRots);

            this.currentGoal = desiredGoal;
        }

        Logger.recordOutput(LogKey + "/CurrentPivotGoal", currentGoal.toString());
        Logger.recordOutput(LogKey + "/DesiredPivotGoal", desiredGoal.toString());
        Logger.recordOutput(
                LogKey + "/PositionSetpoint/PivotPositionRots",
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

    public boolean atGoal(final GroundIntakeArm.Goal goal) {
        return GroundIntakeArm.PositionSetpoint.atSetpoint(
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

    public Translation2d getBoundingBoxSize() {
        return boundingBoxSize;
    }

    public void setGoal(final GroundIntakeArm.Goal goal) {
        this.desiredGoal = goal;
        Logger.recordOutput(LogKey + "/CurrentPivotGoal", currentGoal.toString());
        Logger.recordOutput(LogKey + "/DesiredPivotGoal", desiredGoal.toString());
    }

    public Command runPivotGoalCommand(final GroundIntakeArm.Goal goal) {
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
