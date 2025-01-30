package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.superstructure.arm.elevator.ElevatorArm;
import frc.robot.subsystems.superstructure.arm.intake.IntakeArm;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.utils.solver.SuperstructureSolver;
import frc.robot.utils.subsystems.VirtualSubsystem;
import org.littletonrobotics.junction.Logger;

import java.util.*;
import java.util.function.Supplier;

public class Superstructure extends VirtualSubsystem {
    public enum Goal {
        DYNAMIC(Elevator.Goal.DYNAMIC, ElevatorArm.Goal.DYNAMIC, IntakeArm.PivotGoal.STOW),
        STOW(Elevator.Goal.IDLE, ElevatorArm.Goal.STOW, IntakeArm.PivotGoal.STOW),
        CLIMB(Elevator.Goal.IDLE, ElevatorArm.Goal.CLIMB, IntakeArm.PivotGoal.STOW),
        ALGAE_GROUND(Elevator.Goal.IDLE, ElevatorArm.Goal.UPRIGHT, IntakeArm.PivotGoal.ALGAE_GROUND),
        UPPER_ALGAE(Elevator.Goal.UPPER_ALGAE, ElevatorArm.Goal.UPPER_ALGAE, IntakeArm.PivotGoal.ALGAE_REEF),
        LOWER_ALGAE(Elevator.Goal.LOWER_ALGAE, ElevatorArm.Goal.LOWER_ALGAE, IntakeArm.PivotGoal.ALGAE_REEF),
        HP(Elevator.Goal.HP, ElevatorArm.Goal.UPRIGHT, IntakeArm.PivotGoal.HP),
        PROCESSOR(Elevator.Goal.HP, ElevatorArm.Goal.UPRIGHT, IntakeArm.PivotGoal.HP),
        L1(Elevator.Goal.L1, ElevatorArm.Goal.L1, IntakeArm.PivotGoal.L1),
        L2(Elevator.Goal.L2, ElevatorArm.Goal.L2, IntakeArm.PivotGoal.L2),
        L3(Elevator.Goal.L3, ElevatorArm.Goal.L3, IntakeArm.PivotGoal.L3),
        L4(Elevator.Goal.L4, ElevatorArm.Goal.L4, IntakeArm.PivotGoal.L4),
        NET(Elevator.Goal.NET, ElevatorArm.Goal.UPRIGHT, IntakeArm.PivotGoal.NET);

        private static final Map<Goal, Pose2d> GoalPoses = new HashMap<>();

        static {
            for (final Goal goal : Goal.values()) {
                final Pose2d goalPose = new Pose2d(
                        0.0,
                        goal.elevatorGoal.getPositionGoalMeters(),
                        Rotation2d.fromRotations(goal.elevatorArmGoal.getPivotPositionGoalRots())
                );
                GoalPoses.put(goal, goalPose);
            }
        }

        public final Elevator.Goal elevatorGoal;
        public final ElevatorArm.Goal elevatorArmGoal;
        public final IntakeArm.PivotGoal intakeArmGoal;

        Goal(
                final Elevator.Goal elevatorGoal,
                final ElevatorArm.Goal elevatorArmGoal,
                final IntakeArm.PivotGoal intakeArmGoal
        ) {
            this.elevatorGoal = elevatorGoal;
            this.elevatorArmGoal = elevatorArmGoal;
            this.intakeArmGoal = intakeArmGoal;
        }
    }

    protected static final String LogKey = "Superstructure";

    private final Elevator elevator;
    private final ElevatorArm elevatorArm;
    private final IntakeArm intakeArm;

    private Goal desiredGoal = Goal.STOW;
    private Goal currentGoal = desiredGoal;
    public final Trigger atSuperstructureSetpoint;

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

        this.elevatorArm.setGoal(desiredGoal.elevatorArmGoal);
        this.elevator.setGoal(desiredGoal.elevatorGoal);
        this.intakeArm.setPivotGoal(desiredGoal.intakeArmGoal);
    }

    @Override
    public void periodic() {
        if (desiredGoal != currentGoal) {
            if (desiredGoal != Goal.DYNAMIC) {
                elevatorArm.setGoal(desiredGoal.elevatorArmGoal);
                elevator.setGoal(desiredGoal.elevatorGoal);
                intakeArm.setPivotGoal(desiredGoal.intakeArmGoal);
            }
            this.currentGoal = desiredGoal;
        }

        Logger.recordOutput(
                LogKey + "/Components",
                SuperstructureSolver.calculatePoses(
                        elevatorArm.getPivotPosition(),
                        elevator.getExtensionMeters(),
                        intakeArm.getPivotPosition()
                )
        );

        Logger.recordOutput(LogKey + "/CurrentGoal", currentGoal.toString());
        Logger.recordOutput(LogKey + "/DesiredGoal", desiredGoal.toString());
        Logger.recordOutput(LogKey + "/AtSetpoint", atSuperstructureSetpoint.getAsBoolean());
    }

    public Goal getCurrentSuperstructureGoal() {
        return currentGoal;
    }

    public Goal getDesiredSuperstructureGoal() {
        return desiredGoal;
    }

    public Command runProfile(final SplineProfile profile) {
        final Timer timer = new Timer();
        final Supplier<Pose2d> sampler = profile.sampler(timer::get);

        return Commands.parallel(
                Commands.runOnce(() -> {
                    this.desiredGoal = Goal.DYNAMIC;
                    timer.restart();
                }),
                Commands.run(() -> {
                    Logger.recordOutput(LogKey + "/ProfileTime", timer.get());
                    Logger.recordOutput(LogKey + "/Profile", profile.toString());
                }),
                elevatorArm.toPositionCommand(() -> {
                    final Pose2d sample = sampler.get();
                    return Units.radiansToRotations(sample.getX());
                }),
                elevator.toPositionMetersCommand(() -> {
                    final Pose2d sample = sampler.get();
                    return sample.getY();
                }),
                intakeArm.runPivotGoalCommand(IntakeArm.PivotGoal.STOW)
        ).until(
                atSuperstructureSetpoint
        ).finallyDo(() -> {
            Logger.recordOutput(LogKey + "/Profile", "None");
            timer.stop();
        }).andThen(
                runSuperstructureGoal(profile.endingGoal)
        );
    }

    public Command toInstantSuperstructureGoal(final Goal goal) {
        return Commands.runOnce(
                () -> this.desiredGoal = goal,
                elevator, elevatorArm, intakeArm
        );
    }

    public Command toSuperstructureGoal(final Goal goal) {
        return Commands.runEnd(
                () -> this.desiredGoal = goal,
                () -> this.desiredGoal = Goal.STOW,
                elevator, elevatorArm, intakeArm
        );
    }

    public Command runWaitSuperstructureGoal(final Goal goal) {
        return Commands.run(
                () -> this.desiredGoal = goal,
                elevator, elevatorArm, intakeArm
        ).until(atSuperstructureSetpoint);
    }

    public Command runSuperstructureGoal(final Goal goal) {
        return Commands.run(
                () -> this.desiredGoal = goal,
                elevator, elevatorArm, intakeArm
        );
    }

    public Command runSuperstructureGoal(final Supplier<Goal> goalSupplier) {
        return Commands.run(
                () -> this.desiredGoal = goalSupplier.get(),
                elevator, elevatorArm, intakeArm
        );
    }

    public Set<Subsystem> getRequirements(final Subsystem... subsystems) {
        final Set<Subsystem> requirements = new HashSet<>(Set.of(elevator, elevatorArm, intakeArm));
        if (subsystems.length > 0) {
            requirements.addAll(List.of(subsystems));
        }
        return requirements;
    }

    public Optional<Goal> getClosestGoal() {
        final Pose2d currentPose = new Pose2d(
            0.0,
            elevator.getExtensionMeters(),
            elevatorArm.getPivotPosition()
        );

        Goal closestGoal = null;
        double minDistance = Double.MAX_VALUE;
        for (final Map.Entry<Goal, Pose2d> goalPoseEntry : Goal.GoalPoses.entrySet()) {
            final double distance = goalPoseEntry
                    .getValue()
                    .getTranslation()
                    .getDistance(currentPose.getTranslation());

            if (distance < minDistance) {
                closestGoal = goalPoseEntry.getKey();
                minDistance = distance;
            }
        }

        return Optional.ofNullable(closestGoal);
    }
}
