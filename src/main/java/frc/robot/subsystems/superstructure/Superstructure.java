package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.Constants;
import frc.robot.constants.SimConstants;
import frc.robot.subsystems.superstructure.arm.elevator.ElevatorArm;
import frc.robot.subsystems.superstructure.arm.intake.IntakeArm;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.utils.solver.SuperstructureSimSolver;
import frc.robot.utils.subsystems.VirtualSubsystem;
import org.littletonrobotics.junction.Logger;

import java.util.Set;
import java.util.function.Supplier;

public class Superstructure extends VirtualSubsystem {
    protected static final String LogKey = "Superstructure";

    private final Elevator elevator;
    private final ElevatorArm elevatorArm;
    private final IntakeArm intakeArm;

    private Goal desiredGoal = Goal.IDLE;
    private Goal currentGoal = desiredGoal;
    public final Trigger atSuperstructureSetpoint;

    public enum Goal {
        STOW(Elevator.Goal.IDLE, ElevatorArm.Goal.STOW, IntakeArm.PivotGoal.STOW),
        IDLE(Elevator.Goal.IDLE, ElevatorArm.Goal.UPRIGHT, IntakeArm.PivotGoal.STOW),
        CLIMB(Elevator.Goal.IDLE, ElevatorArm.Goal.CLIMB, IntakeArm.PivotGoal.STOW),
        ALGAE_GROUND(Elevator.Goal.IDLE, ElevatorArm.Goal.UPRIGHT, IntakeArm.PivotGoal.ALGAE_GROUND),
        HP(Elevator.Goal.HP, ElevatorArm.Goal.UPRIGHT, IntakeArm.PivotGoal.HP),
        L1(Elevator.Goal.L1, ElevatorArm.Goal.UPRIGHT, IntakeArm.PivotGoal.L1),
        L2(Elevator.Goal.L2, ElevatorArm.Goal.UPRIGHT, IntakeArm.PivotGoal.L2),
        L3(Elevator.Goal.L3, ElevatorArm.Goal.UPRIGHT, IntakeArm.PivotGoal.L3),
        L4(Elevator.Goal.L4, ElevatorArm.Goal.UPRIGHT, IntakeArm.PivotGoal.L4),
        NET(Elevator.Goal.NET, ElevatorArm.Goal.UPRIGHT, IntakeArm.PivotGoal.NET);

        private final Elevator.Goal elevatorGoal;
        private final ElevatorArm.Goal armGoal;
        private final IntakeArm.PivotGoal intakeArmGoal;

        Goal(
                final Elevator.Goal elevatorGoal,
                final ElevatorArm.Goal armGoal,
                final IntakeArm.PivotGoal intakeArmGoal
        ) {
            this.elevatorGoal = elevatorGoal;
            this.armGoal = armGoal;
            this.intakeArmGoal = intakeArmGoal;
        }
    }

    public Superstructure(
            final Elevator elevator,
            final ElevatorArm elevatorArm,
            final IntakeArm intakeArm
    ) {
        this.elevator = elevator;
        this.elevatorArm = elevatorArm;
        this.intakeArm = intakeArm;
        this.atSuperstructureSetpoint = elevator.atSetpoint
                .and(elevatorArm.atPivotSetpoint)
                .and(intakeArm.atPivotPositionSetpoint)
                .and(() -> currentGoal == desiredGoal);
    }

    @Override
    public void periodic() {
        if (desiredGoal != currentGoal) {
            elevator.setGoal(desiredGoal.elevatorGoal);
            elevatorArm.setGoal(desiredGoal.armGoal);
            intakeArm.setPivotGoal(desiredGoal.intakeArmGoal);
            this.currentGoal = desiredGoal;
        }

        if (Constants.CURRENT_MODE == Constants.RobotMode.SIM) {
            Logger.recordOutput(
                    LogKey + "/Components",
                    SuperstructureSimSolver.calculatePoses(
                            elevatorArm.getPivotPosition(),
                            elevator.getExtensionMeters(),
                            intakeArm.getPivotPosition()
                    )
            );
        }

        Logger.recordOutput(LogKey + "/SuperstructureGoal", currentGoal.toString());
        Logger.recordOutput(LogKey + "/AtSetpoint", atSuperstructureSetpoint.getAsBoolean());
    }

    public Goal getDesiredSuperstructureGoal() {
        return desiredGoal;
    }

    public Command toInstantSuperstructureGoal(final Goal goal) {
        return Commands.runOnce(
                () -> this.desiredGoal = goal
        );
    }
    public Command toSuperstructureGoal(final Goal goal) {
        return Commands.runEnd(() -> this.desiredGoal = goal, () -> this.desiredGoal = Goal.IDLE);
    }

    public Command runSuperstructureGoal(final Goal goal) {
        return Commands.run(() -> this.desiredGoal = goal);
    }

    public Command runSuperstructureGoal(final Supplier<Goal> goalSupplier) {
        return Commands.run(() -> this.desiredGoal = goalSupplier.get());
    }

    public Set<Subsystem> getRequirements() {
        return Set.of(elevator, elevatorArm, intakeArm);
    }

//    public Command intakeHPCoralCommand() {
//        return Commands.sequence(
//                Commands.parallel(
//                        intake.runCoralRollerVelocity(1),
//                        toSuperstructureGoal(SuperstructureGoal.HP)
//                ).until(intake.isCoralPresent),
//                Commands.parallel(
//                        intake.coralInstantStopCommand(),
//                        toSuperstructureGoal(SuperstructureGoal.IDLE)
//                )
//        );
//    }
//
//    public Command intakeAlgaeFromLevelCommand(final SuperstructureGoal superstructureGoal) {
//        return Commands.sequence(
//                Commands.parallel(
//                        intake.runAlgaeRollerVelocity(2),
//                        toSuperstructureGoal(superstructureGoal)
//                ).until(intake.isAlgaePresent),
//                Commands.parallel(
//                        intake.algaeInstantStopCommand(),
//                        toSuperstructureGoal(SuperstructureGoal.IDLE)
//                                .onlyIf(intake.isCoralPresent)
//                )
//        );
//    }
}
