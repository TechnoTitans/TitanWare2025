package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.superstructure.arm.Arm;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.intake.Intake;
import frc.robot.utils.subsystems.VirtualSubsystem;
import org.littletonrobotics.junction.Logger;

public class Superstructure extends VirtualSubsystem {
    protected static final String LogKey = "Superstructure";

    private final Elevator elevator;
    private final Arm arm;
    private final Intake intake;

    private Goal desiredGoal = Goal.IDLE;
    private Goal currentGoal = desiredGoal;
    public final Trigger atSetpoint;

    public enum Goal {
        STOW(Elevator.Goal.IDLE, Arm.Goal.STOW, Intake.PivotGoal.STOW),
        IDLE(Elevator.Goal.IDLE, Arm.Goal.UPRIGHT, Intake.PivotGoal.STOW),
        CLIMB(Elevator.Goal.IDLE, Arm.Goal.CLIMB, Intake.PivotGoal.STOW),
        ALGAE_GROUND(Elevator.Goal.IDLE, Arm.Goal.UPRIGHT, Intake.PivotGoal.ALGAE_GROUND),
        L1(Elevator.Goal.L1, Arm.Goal.UPRIGHT, Intake.PivotGoal.L1),
        L2(Elevator.Goal.L2, Arm.Goal.UPRIGHT, Intake.PivotGoal.L2),
        L3(Elevator.Goal.L3, Arm.Goal.UPRIGHT, Intake.PivotGoal.L3),
        L4(Elevator.Goal.L4, Arm.Goal.UPRIGHT, Intake.PivotGoal.L4),
        NET(Elevator.Goal.NET, Arm.Goal.UPRIGHT, Intake.PivotGoal.NET);

        private final Elevator.Goal elevatorGoal;
        private final Arm.Goal armGoal;
        private final Intake.PivotGoal intakePivotGoal;

        Goal(
                final Elevator.Goal elevatorGoal,
                final Arm.Goal armGoal,
                final Intake.PivotGoal intakePivotGoal
        ) {
            this.elevatorGoal = elevatorGoal;
            this.armGoal = armGoal;
            this.intakePivotGoal = intakePivotGoal;
        }
    }

    public Superstructure(final Elevator elevator, final Arm arm, final Intake intake) {
        this.elevator = elevator;
        this.arm = arm;
        this.intake = intake;
        this.atSetpoint = elevator.atSetpoint
                .and(arm.atPivotSetpoint)
                .and(intake.atPivotPositionSetpoint)
                .and(() -> currentGoal == desiredGoal);
    }

    @Override
    public void periodic() {
        if (desiredGoal != currentGoal) {
            elevator.setGoal(desiredGoal.elevatorGoal);
            arm.setGoal(desiredGoal.armGoal);
            intake.setPivotGoal(desiredGoal.intakePivotGoal);
            this.currentGoal = desiredGoal;
        }

        Logger.recordOutput(LogKey + "/Goal", currentGoal.toString());
        Logger.recordOutput(LogKey + "/AtSetpoint", atSetpoint.getAsBoolean());
    }

    public Goal getDesiredGoal() {
        return desiredGoal;
    }

    public Command toInstantGoal(final Goal goal) {
        return Commands.runOnce(
            () -> this.desiredGoal = goal
        );
    }

    public Command toGoal(final Goal goal) {
        return Commands.runEnd(() -> this.desiredGoal = goal, () -> this.desiredGoal = Goal.IDLE);
    }

    public Command runGoal(final Goal goal) {
        return Commands.run(() -> this.desiredGoal = goal);
    }
}
