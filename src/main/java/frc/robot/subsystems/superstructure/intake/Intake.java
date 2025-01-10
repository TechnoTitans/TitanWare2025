package frc.robot.subsystems.superstructure.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.Constants;
import frc.robot.constants.HardwareConstants;

public class Intake extends SubsystemBase {
    protected static final String LogKey = "Intake";
    private static final double PositionToleranceRots = 0.005;
    private static final double VelocityToleranceRotsPerSec = 0.01;

    private final Constants.RobotMode mode;

    private final IntakeIO intakeIO;
    private final IntakeIOInputsAutoLogged inputs;

    private Goal desiredGoal = Goal.IDLE;
    private Goal currentGoal = desiredGoal;

    private final PositionSetpoint positionSetpoint;
    private final PositionSetpoint pivotLowerLimit;
    private final PositionSetpoint pivotUpperLimit;

    private final VelocitySetpoint velocitySetpoint;

    public final Trigger atPositionSetpoint = new Trigger(this::atPositionSetpoint);
    public final Trigger atLowerLimit = new Trigger(this::atLowerLimit);
    public final Trigger atUpperLimit = new Trigger(this::atUpperLimit);

    public static class PositionSetpoint {
        public double pivotPositionRots = 0.0;

        public PositionSetpoint withPivotPositionRots(final double pivotPositionRots) {
            this.pivotPositionRots = pivotPositionRots;
            return this;
        }

        public boolean atSetpoint(final double pivotPositionRots, final double pivotVelocityRotsPerSec) {
            return MathUtil.isNear(this.pivotPositionRots, pivotPositionRots, PositionToleranceRots)
                    && MathUtil.isNear(0, pivotVelocityRotsPerSec, VelocityToleranceRotsPerSec);
        }
    }

    public static class VelocitySetpoint {
        public double coralRollerVelocityRotsPerSec;
        public double algaeRollerVelocityRotsPerSec;
    }

    public enum Goal {
        NONE(0,0,0),
        IDLE(0,0,0);
        private final double pivotPositionGoalRots;
        private final double coralRollerVelocityRotsPerSec;
        private final double algaeRollerVelocityRotsPerSec;

        Goal(final double pivotPositionGoalRots,
             final double coralRollerVelocityRotsPerSec,
             final double algaeRollerVelocityRotsPerSec) {
            this.pivotPositionGoalRots = pivotPositionGoalRots;
            this.coralRollerVelocityRotsPerSec = coralRollerVelocityRotsPerSec;
            this.algaeRollerVelocityRotsPerSec = algaeRollerVelocityRotsPerSec;
        }

        public double getPivotPositionGoalRots() {
            return pivotPositionGoalRots;
        }

        public double getCoralRollerVelocityRotsPerSec() {
            return coralRollerVelocityRotsPerSec;
        }

        public double getAlgaeRollerVelocityRotsPerSec() {
            return algaeRollerVelocityRotsPerSec;
        }
    }

    public Intake(final Constants.RobotMode mode, final HardwareConstants.IntakeConstants constants) {
        this.mode = mode;

        this.intakeIO = switch (mode) {
            case REAL -> new IntakeIOReal(constants);
            case SIM -> new IntakeIOSim(constants);
            case REPLAY, DISABLED -> new IntakeIO() {};
        };

        this.inputs = new IntakeIOInputsAutoLogged();

        this.positionSetpoint = new PositionSetpoint();
        this.pivotLowerLimit = new PositionSetpoint().withPivotPositionRots(constants.lowerLimitRots());
        this.pivotUpperLimit = new PositionSetpoint().withPivotPositionRots(constants.upperLimitRots());
        this.velocitySetpoint = new VelocitySetpoint();

        this.intakeIO.config();
    }

    @Override
    public void periodic() {
        final double intakePeriodicUpdateStart = RobotController.getFPGATime();

        intakeIO.updateInputs(inputs);

        if (desiredGoal != Goal.NONE && currentGoal != desiredGoal) {
            positionSetpoint.pivotPositionRots = desiredGoal.pivotPositionGoalRots;;
            velocitySetpoint.algaeRollerVelocityRotsPerSec = desiredGoal.algaeRollerVelocityRotsPerSec;
            velocitySetpoint.coralRollerVelocityRotsPerSec = desiredGoal.coralRollerVelocityRotsPerSec;
            this.currentGoal = desiredGoal;
        }
    }

    private boolean atPositionSetpoint() {
        return positionSetpoint.atSetpoint(inputs.pivotPositionRots, inputs.pivotVelocityRotsPerSec)
                && currentGoal == desiredGoal;
    }

    private boolean atLowerLimit() {
        return inputs.pivotPositionRots <= pivotLowerLimit.pivotPositionRots;
    }

    private boolean atUpperLimit() {
        return inputs.pivotPositionRots >= pivotUpperLimit.pivotPositionRots;
    }

    public void setGoal(final Goal goal) {
        this.desiredGoal = goal;
    }
}
