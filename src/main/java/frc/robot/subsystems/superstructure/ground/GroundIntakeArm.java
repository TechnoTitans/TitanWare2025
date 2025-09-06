package frc.robot.subsystems.superstructure.ground;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.Constants;
import frc.robot.constants.HardwareConstants;
import frc.robot.utils.logging.LogUtils;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

public class GroundIntakeArm extends SubsystemBase {
    protected static final String LogKey = "GroundIntakeArm";
    private static final double PositionToleranceRots = 0.1;
    private static final double VelocityToleranceRotsPerSec = 0.1;

    private final GroundIntakeArmIO groundIntakeArmIO;
    private final GroundIntakeArmIOInputsAutoLogged inputs;

    private Goal desiredGoal = Goal.STOW;
    private Goal currentGoal = desiredGoal;

    private final PositionSetpoint setpoint;
    private final PositionSetpoint pivotLowerLimit;
    private final PositionSetpoint pivotUpperLimit;

    public final Trigger atSetpoint = new Trigger(this::atPivotPositionSetpoint);

    public static class PositionSetpoint {
        public double pivotPositionRots = 0;

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
            return PositionSetpoint.atSetpoint(this.pivotPositionRots, pivotPositionRots, pivotVelocityRotsPerSec);
        }
    }

    public enum Goal {
        DYNAMIC(0),
        STOW(0),
        UPRIGHT(-0.09),
        INTAKE_FROM_GROUND(-0.265),
        TRANSFER_CORAL(0.03),
        SCORE_L1(-0.1),
        CLIMB(-.5);

        private final double pivotPositionGoalRots;
        Goal(final double pivotPositionGoalRots) {
            this.pivotPositionGoalRots = pivotPositionGoalRots;
        }

        public double getPivotPositionGoalRots() {
            return pivotPositionGoalRots;
        }
    }

    public GroundIntakeArm(final Constants.RobotMode mode, final HardwareConstants.GroundIntakeArmConstants constants) {
        this.groundIntakeArmIO = switch(mode) {
            case REAL -> new GroundIntakeArmIOReal(constants);
            case SIM -> new GroundIntakeArmIOSim(constants);
            case DISABLED, REPLAY -> new GroundIntakeArmIO() {};
        };

        this.inputs = new GroundIntakeArmIOInputsAutoLogged();

        this.setpoint = new PositionSetpoint().withPivotPositionRots(desiredGoal.getPivotPositionGoalRots());
        this.pivotLowerLimit = new PositionSetpoint().withPivotPositionRots(constants.pivotLowerLimitRots());
        this.pivotUpperLimit = new PositionSetpoint().withPivotPositionRots(constants.pivotUpperLimitRots());

        this.groundIntakeArmIO.config();
        this.groundIntakeArmIO.toPivotPosition(setpoint.pivotPositionRots);
    }

    @Override
    public void periodic() {
        final double groundArmPeriodicUpdateStart = RobotController.getFPGATime();

        groundIntakeArmIO.updateInputs(inputs);
        Logger.processInputs(LogKey, inputs);

        if (desiredGoal != currentGoal) {
            if (desiredGoal != GroundIntakeArm.Goal.DYNAMIC) {
                setpoint.pivotPositionRots = desiredGoal.getPivotPositionGoalRots();
                groundIntakeArmIO.toPivotPosition(setpoint.pivotPositionRots);
            }

            this.currentGoal = desiredGoal;

            Logger.recordOutput(LogKey + "/CurrentGoal", currentGoal.toString());
            Logger.recordOutput(LogKey + "/DesiredGoal", desiredGoal.toString());
            Logger.recordOutput(LogKey + "/PositionSetpoint/PivotPositionRots", setpoint.pivotPositionRots);
            Logger.recordOutput(LogKey + "/AtPositionSetpoint", atPivotPositionSetpoint());
            Logger.recordOutput(LogKey + "/AtPivotLowerLimit", atPivotLowerLimit());
            Logger.recordOutput(LogKey + "/AtPivotUpperLimit", atPivotUpperLimit());

            Logger.recordOutput(
                    LogKey + "/PeriodicIOPeriodMs",
                    LogUtils.microsecondsToMilliseconds(RobotController.getFPGATime() - groundArmPeriodicUpdateStart)
            );
        }
    }

    private boolean atPivotPositionSetpoint() {
        return setpoint.atSetpoint(inputs.pivotPositionRots, inputs.pivotVelocityRotsPerSec)
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
        Logger.recordOutput(LogKey + "/CurrentGoal", currentGoal.toString());
        Logger.recordOutput(LogKey + "/DesiredGoal", desiredGoal.toString());
    }
}
