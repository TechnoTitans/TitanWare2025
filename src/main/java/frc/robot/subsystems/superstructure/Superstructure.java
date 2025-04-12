package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.superstructure.distal.IntakeArm;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.proximal.ElevatorArm;
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
        NONE(Elevator.Goal.STOW, ElevatorArm.Goal.STOW, IntakeArm.Goal.STOW),
        DYNAMIC(Elevator.Goal.DYNAMIC, ElevatorArm.Goal.DYNAMIC, IntakeArm.Goal.STOW),
        UNSTAND(Elevator.Goal.STOW, ElevatorArm.Goal.UNSTAND, IntakeArm.Goal.STOW),
        STOW(Elevator.Goal.STOW, ElevatorArm.Goal.STOW, IntakeArm.Goal.STOW),
        CLIMB(Elevator.Goal.STOW, ElevatorArm.Goal.CLIMB, IntakeArm.Goal.CLIMB),
        CLIMB_DOWN(Elevator.Goal.CLIMB_DOWN, ElevatorArm.Goal.CLIMB_DOWN, IntakeArm.Goal.CLIMB_DOWN),
        ALGAE_GROUND(Elevator.Goal.ALGAE_GROUND, ElevatorArm.Goal.ALGAE_GROUND, IntakeArm.Goal.ALGAE_GROUND),
        UPPER_ALGAE(Elevator.Goal.UPPER_ALGAE, ElevatorArm.Goal.UPPER_ALGAE, IntakeArm.Goal.UPPER_ALGAE),
        LOWER_ALGAE(Elevator.Goal.LOWER_ALGAE, ElevatorArm.Goal.LOWER_ALGAE, IntakeArm.Goal.LOWER_ALGAE),
        HP(Elevator.Goal.HP, ElevatorArm.Goal.HP, IntakeArm.Goal.HP),
        PROCESSOR(Elevator.Goal.PROCESSOR, ElevatorArm.Goal.PROCESSOR, IntakeArm.Goal.PROCESSOR),
        L1(Elevator.Goal.L1, ElevatorArm.Goal.L1, IntakeArm.Goal.L1),
        ALIGN_L1(Elevator.Goal.L1, ElevatorArm.Goal.L1, IntakeArm.Goal.L1),
        L2(Elevator.Goal.L2, ElevatorArm.Goal.L2, IntakeArm.Goal.L2),
        ALIGN_L2(Elevator.Goal.L2, ElevatorArm.Goal.L2, IntakeArm.Goal.L2),
        L3(Elevator.Goal.L3, ElevatorArm.Goal.L3, IntakeArm.Goal.L3),
        ALIGN_L3(Elevator.Goal.STOW, ElevatorArm.Goal.L3, IntakeArm.Goal.L3),
        L4(Elevator.Goal.L4, ElevatorArm.Goal.L4, IntakeArm.Goal.L4),
        AUTO_L4(Elevator.Goal.AUTO_L4, ElevatorArm.Goal.AUTO_L4, IntakeArm.Goal.AUTO_L4),
        ALIGN_L4(Elevator.Goal.STOW, ElevatorArm.Goal.L4, IntakeArm.Goal.L4),
        NET(Elevator.Goal.NET, ElevatorArm.Goal.UPRIGHT, IntakeArm.Goal.NET),
        ALIGN_NET(Elevator.Goal.STOW, ElevatorArm.Goal.UPRIGHT, IntakeArm.Goal.NET),
        FLING_NET(Elevator.Goal.NET, ElevatorArm.Goal.UPRIGHT, IntakeArm.Goal.ALGAE_FLING),

        SAFE(Elevator.Goal.L3, ElevatorArm.Goal.L4, IntakeArm.Goal.STOW);

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

        public static Goal getAlignGoal(final Goal goal) {
            return switch (goal) {
                case L1 -> ALIGN_L1;
                case L2 -> ALIGN_L2;
                case L3 -> ALIGN_L3;
                case L4 -> ALIGN_L4;
                default -> STOW;
            };
        }
    }

    protected static final String LogKey = "Superstructure";
    public static final double AllowableExtensionForDrivingMeters =
            Goal.GoalTranslations.get(Goal.SAFE).getNorm();

    private final Elevator elevator;
    private final ElevatorArm elevatorArm;
    private final IntakeArm intakeArm;

    //TODO: change to stow right after this has happened
    private Goal desiredGoal = Goal.UNSTAND;
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

    private final Trigger desiredGoalNotStow;
    private final Trigger atSuperstructureSetpoint;

    public final Trigger unsafeToDrive;

    public Superstructure(
            final ElevatorArm elevatorArm,
            final Elevator elevator,
            final IntakeArm intakeArm
    ) {
        this.elevatorArm = elevatorArm;
        this.elevator = elevator;
        this.intakeArm = intakeArm;

        this.eventLoop = new EventLoop();

        this.desiredGoalIsRunningGoal = new Trigger(eventLoop, () -> desiredGoal == runningGoal);
        this.desiredGoalChanged = new Trigger(eventLoop, () -> desiredGoal != runningGoal);
        this.desiredGoalIsAtGoal = new Trigger(eventLoop, () -> desiredGoal == atGoal);
        this.desiredGoalIsDynamic = new Trigger(eventLoop, () -> desiredGoal == Goal.DYNAMIC);
        this.desiredGoalNotStow = new Trigger(eventLoop, () -> desiredGoal != Goal.STOW);
        this.atSuperstructureSetpoint = elevator.atSetpoint
                .and(elevatorArm.atSetpoint)
                .and(intakeArm.atSetpoint)
                .and(desiredGoalIsAtGoal);

        this.unsafeToDrive = extendedBeyond(AllowableExtensionForDrivingMeters);

        this.allowedToChangeGoal = desiredGoalIsDynamic.negate()
                .and((desiredGoalIsAtGoal.and(atSuperstructureSetpoint)).negate());
        this.desiresUpwardsMotion = new Trigger(eventLoop, () -> {
            final Translation2d currentTranslation = getCurrentTranslation();
            final Translation2d desiredTranslation = Superstructure.Goal.GoalTranslations.get(desiredGoal);

            return desiredTranslation.getY() >= currentTranslation.getY();
        });
        this.desiresDownwardsMotion = desiresUpwardsMotion.negate();

        final Command upwardsGoalChange = upwardsGoalChange();
        desiredGoalChanged.and(allowedToChangeGoal).and(desiresUpwardsMotion)
                .onTrue(Commands.runOnce(() -> {
                    upwardsGoalChange.cancel();
                    upwardsGoalChange.schedule();
                }));

        final Command downwardsGoalChange = downwardsGoalChange();
        desiredGoalChanged.and(allowedToChangeGoal).and(desiresDownwardsMotion)
                .onTrue(Commands.runOnce(() -> {
                    downwardsGoalChange.cancel();
                    downwardsGoalChange.schedule();
                }));

        elevatorArm.setGoal(desiredGoal.elevatorArmGoal);
        elevator.setGoal(desiredGoal.elevatorGoal);
        intakeArm.setGoal(desiredGoal.intakeArmGoal);
    }

    private Command upwardsGoalChange() {
        return Commands.sequence(
                Commands.runOnce(() -> {
                    this.atGoal = Goal.NONE;
                    this.runningGoal = desiredGoal;
                }),
                Commands.runOnce(() -> elevatorArm.setGoal(runningGoal.elevatorArmGoal)),
                Commands.runOnce(() -> intakeArm.setGoal(runningGoal.intakeArmGoal)),

                Commands.waitUntil(elevatorArm.atSetpoint.and(intakeArm.atSetpoint))
                        .withTimeout(4),
                Commands.runOnce(() -> elevator.setGoal(runningGoal.elevatorGoal)),

                Commands.waitUntil(
                        elevatorArm.atSetpoint
                                .and(elevator.atSetpoint)
                                .and(intakeArm.atSetpoint)).withTimeout(4),
                Commands.runOnce(() -> this.atGoal = runningGoal)
        )
                .onlyWhile(() -> desiredGoal == runningGoal)
                .withName("UpwardsGoalChange");
    }

    private Command downwardsGoalChange() {
        return Commands.sequence(
                Commands.runOnce(() -> {
                    this.atGoal = Goal.NONE;
                    this.runningGoal = desiredGoal;
                }),
                Commands.runOnce(() -> intakeArm.setGoal(runningGoal.intakeArmGoal)),
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
        )
                .onlyWhile(() -> desiredGoal == runningGoal)
                .withName("DownwardsGoalChange");
    }

    private Command goalChange() {
        return Commands.parallel(
                upwardsGoalChange()
                        .onlyIf(desiresUpwardsMotion)
                        .onlyWhile(desiresUpwardsMotion),
                downwardsGoalChange()
                        .onlyIf(desiresDownwardsMotion)
                        .onlyWhile(desiresDownwardsMotion)
        ).withName("GoalChange");
    }

    private Command runOnce(final Runnable action) {
        return Commands.runOnce(action, elevator, elevatorArm, intakeArm);
    }

    private Command runEnd(final Runnable run, final Runnable end) {
        return Commands.runEnd(run, end, elevator, elevatorArm, intakeArm);
    }

    private Command run(final Runnable run) {
        return Commands.run(run, elevator, elevatorArm, intakeArm);
    }

    private Runnable setDesiredGoal(final Supplier<Goal> desiredGoalSupplier) {
        return () -> {
            this.desiredGoal = desiredGoalSupplier.get();
            eventLoop.poll();
        };
    }

    private Runnable setDesiredGoal(final Goal desiredGoal) {
        return setDesiredGoal(() -> desiredGoal);
    }

    @Override
    public void periodic() {
        eventLoop.poll();

        Logger.recordOutput(LogKey + "/RunningGoal", runningGoal.toString());
        Logger.recordOutput(LogKey + "/DesiredGoal", desiredGoal.toString());
        Logger.recordOutput(LogKey + "/AtGoal", atGoal.toString());

        Logger.recordOutput(LogKey + "/DesiredGoalNotStow", desiredGoalNotStow);
        Logger.recordOutput(LogKey + "/AtSetpoint", atSuperstructureSetpoint);
        Logger.recordOutput(LogKey + "/UnsafeToDrive", unsafeToDrive);

        Logger.recordOutput(LogKey + "/ExtensionDistanceMeters", getCurrentTranslation().getNorm());
        Logger.recordOutput(LogKey + "/AllowableExtensionForDrivingMeters", AllowableExtensionForDrivingMeters);

        Logger.recordOutput(LogKey + "/Triggers/DesiredGoalIsRunningGoal", desiredGoalIsRunningGoal);
        Logger.recordOutput(LogKey + "/Triggers/DesiredGoalIsAtGoal", desiredGoalIsAtGoal);
        Logger.recordOutput(LogKey + "/Triggers/DesiredGoalIsDynamic", desiredGoalIsDynamic);
        Logger.recordOutput(LogKey + "/Triggers/AllowedToChangeGoal", allowedToChangeGoal);
        Logger.recordOutput(LogKey + "/Triggers/DesiresUpwardsMotion", desiresUpwardsMotion);
        Logger.recordOutput(LogKey + "/Triggers/DesiresDownwardsMotion", desiresDownwardsMotion);
        Logger.recordOutput(LogKey + "/Triggers/DesiredGoalChanged", desiredGoalChanged);

        Logger.recordOutput(LogKey + "/Components", getComponentPoses());
    }

    public Pose3d[] getComponentPoses() {
        return SuperstructureSolver.calculatePoses(
                elevatorArm.getPivotPosition(),
                elevator.getExtensionMeters(),
                intakeArm.getPivotPosition()
        );
    }

    public Trigger atSetpoint(final Supplier<Superstructure.Goal> goalSupplier) {
        return atSuperstructureSetpoint.and(() -> atGoal == goalSupplier.get());
    }

    public Trigger atSetpoint(final Superstructure.Goal goal) {
        return atSetpoint(() -> goal);
    }

    public Trigger extendedBeyond(final double distance) {
        return new Trigger(eventLoop, () -> getCurrentTranslation().getNorm() > distance);
    }

    public Command forceGoal(final Goal goal) {
        return runOnce(setDesiredGoal(goal))
                .andThen(goalChange())
                .withName("ForceGoal: " + goal);
    }

    public Command toInstantGoal(final Goal goal) {
        return runOnce(setDesiredGoal(goal))
                .withName("ToInstantGoal: " + goal);
    }

    public Command toGoal(final Goal goal) {
        return runEnd(
                setDesiredGoal(goal),
                setDesiredGoal(Goal.STOW)
        ).withName("ToGoal: " + goal);
    }

    public Command toGoal(final Supplier<Goal> goal) {
        return runEnd(
                setDesiredGoal(goal),
                setDesiredGoal(Goal.STOW)
        ).withName("ToGoal");
    }

    public Command runGoal(final Goal goal) {
        return run(setDesiredGoal(goal))
                .withName("RunGoal: " + goal);
    }

    public Command runGoal(final Supplier<Goal> goalSupplier) {
        return run(setDesiredGoal(goalSupplier))
                .withName("RunGoal");
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