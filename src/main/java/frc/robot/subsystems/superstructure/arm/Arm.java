package frc.robot.subsystems.superstructure.arm;

import com.ctre.phoenix6.hardware.CANrange;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.Constants;
import frc.robot.constants.HardwareConstants;
import frc.robot.utils.logging.LogUtils;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
    protected static final String LogKey = "Arm";
    private static final double PositionToleranceRots = 0.005;
    private static final double VelocityToleranceRotsPerSec = 0.01;

    private final ArmIO armIO;
    private final ArmIOInputsAutoLogged inputs;

//    private final SysIdRoutine voltageSysIdRoutine;
//    private final SysIdRoutine torqueCurrentSysIdRoutine;

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
}
