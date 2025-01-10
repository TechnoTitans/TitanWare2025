package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.superstructure.arm.Arm;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.intake.Intake;
import frc.robot.utils.subsystems.VirtualSubsystem;
import org.littletonrobotics.junction.Logger;

import java.util.Set;

public class Superstructure extends VirtualSubsystem {
    protected static final String LogKey = "Superstructure";

    private final Elevator elevator;
    private final Arm arm;
    private final Intake intake;

    private SuperstructureGoal desiredSuperstructureGoal = SuperstructureGoal.IDLE;
    private SuperstructureGoal currentSuperstructureGoal = desiredSuperstructureGoal;
    public final Trigger atSuperstructureSetpoint;

    public enum SuperstructureGoal {
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

        SuperstructureGoal(
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
        this.atSuperstructureSetpoint = elevator.atSetpoint
                .and(arm.atPivotSetpoint)
                .and(intake.atPivotPositionSetpoint)
                .and(() -> currentSuperstructureGoal == desiredSuperstructureGoal);
    }

    @Override
    public void periodic() {
        if (desiredSuperstructureGoal != currentSuperstructureGoal) {
            elevator.setGoal(desiredSuperstructureGoal.elevatorGoal);
            arm.setGoal(desiredSuperstructureGoal.armGoal);
            intake.setPivotGoal(desiredSuperstructureGoal.intakePivotGoal);
            this.currentSuperstructureGoal = desiredSuperstructureGoal;
        }

        Logger.recordOutput(LogKey + "/SuperstructureGoal", currentSuperstructureGoal.toString());
        Logger.recordOutput(LogKey + "/AtSetpoint", atSuperstructureSetpoint.getAsBoolean());
    }

    public SuperstructureGoal getDesiredSuperstructureGoal() {
        return desiredSuperstructureGoal;
    }

    public Command toInstantSuperstructureGoal(final SuperstructureGoal superstructureGoal) {
        return Commands.runOnce(
            () -> this.desiredSuperstructureGoal = superstructureGoal
        );
    }
    public Command toSuperstructureGoal(final SuperstructureGoal superstructureGoal) {
        return Commands.runEnd(() -> this.desiredSuperstructureGoal = superstructureGoal, () -> this.desiredSuperstructureGoal = SuperstructureGoal.IDLE);
    }
    public Command runSuperstructureGoal(final SuperstructureGoal superstructureGoal) {
        return Commands.run(() -> this.desiredSuperstructureGoal = superstructureGoal);
    }

    public Command toInstantIntakeRollerGoal(final Intake.RollerGoal rollerGoal) {
        return Commands.runOnce(
            () -> this.intake.setRollerGoal(rollerGoal)
        );
    }
    public Command toIntakeRollerGoal(final Intake.RollerGoal rollerGoal) {
        return Commands.runEnd(() -> this.intake.setRollerGoal(rollerGoal), () -> this.intake.setRollerGoal(Intake.RollerGoal.STOP));
    }
    public Command runIntakeRollerGoal(final Intake.RollerGoal rollerGoal) {
        return Commands.run(() -> this.intake.setRollerGoal(rollerGoal));
    }

    public Set<Subsystem> getRequirements() {
        return Set.of(elevator, arm, intake);
    }
}
