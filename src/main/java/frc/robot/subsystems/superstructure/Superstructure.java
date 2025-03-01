package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.superstructure.arm.elevator.ElevatorArm;
import frc.robot.subsystems.superstructure.arm.intake.IntakeArm;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.utils.solver.SuperstructureSolver;
import frc.robot.utils.subsystems.VirtualSubsystem;
import org.littletonrobotics.junction.Logger;

import java.util.HashMap;
import java.util.Map;
import java.util.Optional;
import java.util.Set;
import java.util.function.Supplier;

public class Superstructure extends VirtualSubsystem {
    public enum Goal {
        DYNAMIC(Elevator.Goal.DYNAMIC, ElevatorArm.Goal.DYNAMIC, IntakeArm.Goal.STOW),
        STOW(Elevator.Goal.STOW, ElevatorArm.Goal.STOW, IntakeArm.Goal.STOW),
        CLIMB(Elevator.Goal.STOW, ElevatorArm.Goal.CLIMB, IntakeArm.Goal.CLIMB),
        CLIMB_DOWN(Elevator.Goal.STOW, ElevatorArm.Goal.CLIMB_DOWN, IntakeArm.Goal.CLIMB),
        ALGAE_GROUND(Elevator.Goal.ALGAE_GROUND, ElevatorArm.Goal.ALGAE_GROUND, IntakeArm.Goal.ALGAE_GROUND),
        UPPER_ALGAE(Elevator.Goal.UPPER_ALGAE, ElevatorArm.Goal.UPPER_ALGAE, IntakeArm.Goal.UPPER_ALGAE),
        LOWER_ALGAE(Elevator.Goal.LOWER_ALGAE, ElevatorArm.Goal.LOWER_ALGAE, IntakeArm.Goal.LOWER_ALGAE),
        HP(Elevator.Goal.HP, ElevatorArm.Goal.HP, IntakeArm.Goal.HP),
        PROCESSOR(Elevator.Goal.PROCESSOR, ElevatorArm.Goal.PROCESSOR, IntakeArm.Goal.PROCESSOR),
        L1(Elevator.Goal.L1, ElevatorArm.Goal.L1, IntakeArm.Goal.L1),
        L2(Elevator.Goal.L2, ElevatorArm.Goal.L2, IntakeArm.Goal.L2),
        L3(Elevator.Goal.L3, ElevatorArm.Goal.L3, IntakeArm.Goal.L3),
        L4(Elevator.Goal.L4, ElevatorArm.Goal.L4, IntakeArm.Goal.L4),
        NET(Elevator.Goal.NET, ElevatorArm.Goal.UPRIGHT, IntakeArm.Goal.NET),

        SAFE(Elevator.Goal.L3, ElevatorArm.Goal.L3, IntakeArm.Goal.STOW);

        private static final Map<Goal, Translation2d> GoalTranslations = new HashMap<>();

        static {
            for (final Goal goal : Superstructure.Goal.values()) {
                final Translation2d goalTranslation = new Translation2d(
                        goal.elevatorGoal.getPositionGoalMeters(),
                        Rotation2d.fromRotations(goal.elevatorArmGoal.getPivotPositionGoalRots())
                );
                GoalTranslations.put(goal, goalTranslation);
            }
        }

        public final Elevator.Goal elevatorGoal;
        public final ElevatorArm.Goal elevatorArmGoal;
        public final IntakeArm.Goal intakeArmGoal;

        Goal(
                final Elevator.Goal elevatorGoal,
                final ElevatorArm.Goal elevatorArmGoal,
                final IntakeArm.Goal intakeArmGoal
        ) {
            this.elevatorGoal = elevatorGoal;
            this.elevatorArmGoal = elevatorArmGoal;
            this.intakeArmGoal = intakeArmGoal;
        }
    }

    protected static final String LogKey = "Superstructure";
    public static final double AllowableExtensionForDrivingMeters =
            Goal.GoalTranslations.get(Goal.SAFE).getNorm();

    private final Elevator elevator;
    private final ElevatorArm elevatorArm;
    private final IntakeArm intakeArm;

    private Goal desiredGoal = Superstructure.Goal.STOW;
    private Goal runningGoal = desiredGoal;
    private Goal atGoal = desiredGoal;

    private final EventLoop eventLoop;

    private final Trigger desiredGoalIsRunningGoal;
    private final Trigger desiredGoalIsAtGoal;
    private final Trigger desiredGoalIsDynamic;

    private final Trigger allowedToChangeGoal;

    private final Trigger desiresUpwardsMotion;
    private final Trigger desiresDownwardsMotion;
    private final Trigger desiredGoalChanged;

    public final Trigger desiredGoalNotStow;
    public final Trigger atSuperstructureSetpoint;

    public final Trigger unsafeToDrive;

    public Superstructure(
            final Elevator elevator,
            final ElevatorArm elevatorArm,
            final IntakeArm intakeArm
    ) {
        this.elevator = elevator;
        this.elevatorArm = elevatorArm;
        this.intakeArm = intakeArm;

        this.eventLoop = new EventLoop();

        this.desiredGoalIsRunningGoal = new Trigger(eventLoop, () -> desiredGoal == runningGoal);
        this.desiredGoalChanged = new Trigger(eventLoop, () -> desiredGoal != runningGoal);
        this.desiredGoalIsAtGoal = new Trigger(eventLoop, () -> desiredGoal == atGoal);
        this.desiredGoalIsDynamic = new Trigger(eventLoop, () -> desiredGoal == Superstructure.Goal.DYNAMIC);
        this.desiredGoalNotStow = new Trigger(eventLoop, () -> desiredGoal != Goal.STOW);
        this.atSuperstructureSetpoint = elevator.atSetpoint
                .and(elevatorArm.atSetpoint)
                .and(intakeArm.atSetpoint)
                .and(desiredGoalIsAtGoal);

        this.unsafeToDrive = new Trigger(eventLoop, () ->
                getCurrentTranslation().getNorm() > AllowableExtensionForDrivingMeters);

        this.allowedToChangeGoal = desiredGoalIsDynamic.negate()
                .and((desiredGoalIsAtGoal.and(atSuperstructureSetpoint)).negate());
        this.desiresUpwardsMotion = new Trigger(eventLoop, () -> {
            final Translation2d currentTranslation = getCurrentTranslation();
            final Translation2d desiredTranslation = Superstructure.Goal.GoalTranslations.get(desiredGoal);

            return desiredTranslation.getY() >= currentTranslation.getY();
        });
        this.desiresDownwardsMotion = desiresUpwardsMotion.negate();

        desiredGoalChanged.and(allowedToChangeGoal).and(desiresUpwardsMotion).onTrue(
                Commands.sequence(
                        Commands.runOnce(() -> this.runningGoal = desiredGoal),
                        Commands.sequence(
                                Commands.runOnce(() -> elevatorArm.setGoal(runningGoal.elevatorArmGoal)),

                                Commands.waitUntil(elevatorArm.atSetpoint).withTimeout(4),
                                Commands.runOnce(() -> elevator.setGoal(runningGoal.elevatorGoal)),

                                Commands.waitUntil(
                                        elevatorArm.atSetpoint
                                                .and(elevator.atSetpoint)).withTimeout(4),
                                Commands.runOnce(() -> intakeArm.setGoal(runningGoal.intakeArmGoal)),

                                Commands.waitUntil(
                                        elevatorArm.atSetpoint
                                                .and(elevator.atSetpoint)
                                                .and(intakeArm.atSetpoint)).withTimeout(4),
                                Commands.runOnce(() -> this.atGoal = runningGoal)
                        ).onlyWhile(() -> desiredGoal == runningGoal)
                ).withName("UpwardsGoalChange")
        );

        desiredGoalChanged.and(allowedToChangeGoal).and(desiresDownwardsMotion).onTrue(
                Commands.sequence(
                        Commands.runOnce(() -> this.runningGoal = desiredGoal),
                        Commands.sequence(
                                Commands.runOnce(() -> intakeArm.setGoal(runningGoal.intakeArmGoal)),

                                Commands.waitUntil(intakeArm.atSetpoint).withTimeout(4),
                                Commands.runOnce(() -> elevator.setGoal(runningGoal.elevatorGoal)),

                                Commands.waitUntil(
                                        elevator.atSetpoint
                                                .and(intakeArm.atSetpoint)).withTimeout(4),
                                Commands.runOnce(() -> elevatorArm.setGoal(runningGoal.elevatorArmGoal)),

                                Commands.waitUntil(
                                        elevatorArm.atSetpoint
                                                .and(elevator.atSetpoint)
                                                .and(intakeArm.atSetpoint)).withTimeout(4),
                                Commands.runOnce(() -> this.atGoal = runningGoal)
                        ).onlyWhile(() -> desiredGoal == runningGoal)
                ).withName("DownwardsGoalChange")
        );

        elevatorArm.setGoal(desiredGoal.elevatorArmGoal);
        elevator.setGoal(desiredGoal.elevatorGoal);
        intakeArm.setGoal(desiredGoal.intakeArmGoal);
    }

    @Override
    public void periodic() {
        eventLoop.poll();

        Logger.recordOutput(LogKey + "/RunningGoal", runningGoal.toString());
        Logger.recordOutput(LogKey + "/DesiredGoal", desiredGoal.toString());
        Logger.recordOutput(LogKey + "/AtGoal", atGoal.toString());
        Logger.recordOutput(LogKey + "/AtSetpoint", atSuperstructureSetpoint.getAsBoolean());
        Logger.recordOutput(LogKey + "/Triggers/DesiredGoalIsRunningGoal", desiredGoalIsRunningGoal);
        Logger.recordOutput(LogKey + "/Triggers/DesiredGoalIsAtGoal", desiredGoalIsAtGoal);
        Logger.recordOutput(LogKey + "/Triggers/DesiredGoalIsDynamic", desiredGoalIsDynamic);
        Logger.recordOutput(LogKey + "/Triggers/AllowedToChangeGoal", allowedToChangeGoal);
        Logger.recordOutput(LogKey + "/Triggers/DesiresUpwardsMotion", desiresUpwardsMotion);
        Logger.recordOutput(LogKey + "/Triggers/DesiresDownwardsMotion", desiresDownwardsMotion);
        Logger.recordOutput(LogKey + "/Triggers/DesiredGoalChanged", desiredGoalChanged);

        Logger.recordOutput(
                LogKey + "/Components",
                SuperstructureSolver.calculatePoses(
                        elevatorArm.getPivotPosition(),
                        elevator.getExtensionMeters(),
                        intakeArm.getPivotPosition()
                )
        );
    }

    public Goal getDesiredSuperstructureGoal() {
        return desiredGoal;
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
                () -> this.desiredGoal = Superstructure.Goal.STOW,
                elevator, elevatorArm, intakeArm
        );
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

    public Set<Subsystem> getRequirements() {
        return Set.of(elevator, elevatorArm, intakeArm);
    }

    private Translation2d getCurrentTranslation() {
        return new Translation2d(
                elevator.getExtensionMeters(),
                elevatorArm.getPivotPosition()
        );
    }

    @SuppressWarnings("unused")
    public Optional<Goal> getClosestGoal(final Set<Goal> goalWhitelist) {
        final Translation2d currentTranslation = getCurrentTranslation();

        Goal closestGoal = null;
        double minDistance = Double.MAX_VALUE;
        for (final Map.Entry<Goal, Translation2d> goalTranslationEntry
                : Superstructure.Goal.GoalTranslations.entrySet()
        ) {
            final Goal goal = goalTranslationEntry.getKey();
            if (!goalWhitelist.contains(goal)) {
                continue;
            }

            final Translation2d goalTranslation = goalTranslationEntry.getValue();
            final double distance = goalTranslation.getDistance(currentTranslation);

            if (distance < minDistance) {
                closestGoal = goal;
                minDistance = distance;
            }
        }

        return Optional.ofNullable(closestGoal);
    }
}