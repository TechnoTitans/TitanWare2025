package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.superstructure.arm.elevator.ElevatorArm;
import frc.robot.subsystems.superstructure.arm.intake.IntakeArm;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.utils.subsystems.VirtualSubsystem;
import org.littletonrobotics.junction.Logger;

import java.util.Set;

public class Superstructure extends VirtualSubsystem {
    protected static final String LogKey = "Superstructure";

    private final Elevator elevator;
    private final ElevatorArm elevatorArm;
    private final IntakeArm intakeArm;

    private SuperstructureGoal desiredSuperstructureGoal = SuperstructureGoal.IDLE;
    private SuperstructureGoal currentSuperstructureGoal = desiredSuperstructureGoal;
    public final Trigger atSuperstructureSetpoint;

    public enum SuperstructureGoal {
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

        SuperstructureGoal(
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
                .and(() -> currentSuperstructureGoal == desiredSuperstructureGoal);
    }

    @Override
    public void periodic() {
        if (desiredSuperstructureGoal != currentSuperstructureGoal) {
            elevator.setGoal(desiredSuperstructureGoal.elevatorGoal);
            elevatorArm.setGoal(desiredSuperstructureGoal.armGoal);
            intakeArm.setPivotGoal(desiredSuperstructureGoal.intakeArmGoal);
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
