package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.superstructure.distal.IntakeArm;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.ground.GroundIntakeArm;
import frc.robot.subsystems.superstructure.proximal.ElevatorArm;
import frc.robot.utils.Container;
import frc.robot.utils.commands.FastCommands;
import frc.robot.utils.commands.LoggedTrigger;
import frc.robot.utils.geometry.MutableEllipse2d;
import frc.robot.utils.subsystems.VirtualSubsystem;
import org.littletonrobotics.junction.Logger;

import java.util.HashMap;
import java.util.Map;
import java.util.Optional;
import java.util.Set;
import java.util.function.Supplier;

public class Superstructure extends VirtualSubsystem {
    public enum Goal {
        NONE(Elevator.Goal.STOW, ElevatorArm.Goal.STOW, IntakeArm.Goal.STOW, GroundIntakeArm.Goal.STOW),
        DYNAMIC(Elevator.Goal.DYNAMIC, ElevatorArm.Goal.DYNAMIC, IntakeArm.Goal.STOW, GroundIntakeArm.Goal.STOW),
        STOW(Elevator.Goal.STOW, ElevatorArm.Goal.STOW, IntakeArm.Goal.STOW, GroundIntakeArm.Goal.STOW),

        HANDOFF(Elevator.Goal.HANDOFF, ElevatorArm.Goal.HANDOFF, IntakeArm.Goal.HANDOFF, GroundIntakeArm.Goal.HANDOFF),

        CLIMB(Elevator.Goal.STOW, ElevatorArm.Goal.CLIMB, IntakeArm.Goal.CLIMB, GroundIntakeArm.Goal.STOW),
        CLIMB_DOWN(Elevator.Goal.CLIMB_DOWN, ElevatorArm.Goal.CLIMB_DOWN, IntakeArm.Goal.CLIMB_DOWN, GroundIntakeArm.Goal.STOW),

        ALGAE_GROUND(Elevator.Goal.ALGAE_GROUND, ElevatorArm.Goal.ALGAE_GROUND, IntakeArm.Goal.ALGAE_GROUND, GroundIntakeArm.Goal.STOW),
        UPPER_ALGAE(Elevator.Goal.UPPER_ALGAE, ElevatorArm.Goal.UPPER_ALGAE, IntakeArm.Goal.UPPER_ALGAE, GroundIntakeArm.Goal.ALGAE_SAFE),
        LOWER_ALGAE(Elevator.Goal.LOWER_ALGAE, ElevatorArm.Goal.LOWER_ALGAE, IntakeArm.Goal.LOWER_ALGAE, GroundIntakeArm.Goal.ALGAE_SAFE),

        HP(Elevator.Goal.HP, ElevatorArm.Goal.HP, IntakeArm.Goal.HP, GroundIntakeArm.Goal.STOW),
        GROUND_INTAKE(Elevator.Goal.HANDOFF, ElevatorArm.Goal.HANDOFF, IntakeArm.Goal.HANDOFF, GroundIntakeArm.Goal.INTAKE),
        PROCESSOR(Elevator.Goal.PROCESSOR, ElevatorArm.Goal.PROCESSOR, IntakeArm.Goal.PROCESSOR, GroundIntakeArm.Goal.STOW),

        L1(Elevator.Goal.L1, ElevatorArm.Goal.L1, IntakeArm.Goal.L1, GroundIntakeArm.Goal.INTAKE),
        ALIGN_L1(Elevator.Goal.L1, ElevatorArm.Goal.L1, IntakeArm.Goal.L1, GroundIntakeArm.Goal.STOW),
        L2(Elevator.Goal.L2, ElevatorArm.Goal.L2, IntakeArm.Goal.L2, GroundIntakeArm.Goal.STOW),
        ALIGN_L2(Elevator.Goal.L2, ElevatorArm.Goal.L2, IntakeArm.Goal.L2, GroundIntakeArm.Goal.STOW),
        L3(Elevator.Goal.L3, ElevatorArm.Goal.L3, IntakeArm.Goal.L3, GroundIntakeArm.Goal.STOW),
        ALIGN_L3(Elevator.Goal.STOW, ElevatorArm.Goal.L3, IntakeArm.Goal.L3, GroundIntakeArm.Goal.STOW),
        L4(Elevator.Goal.L4, ElevatorArm.Goal.L4, IntakeArm.Goal.L4, GroundIntakeArm.Goal.STOW),
        ALIGN_L4(Elevator.Goal.STOW, ElevatorArm.Goal.L4, IntakeArm.Goal.L4, GroundIntakeArm.Goal.STOW),
        ALIGN_AUTO_L4(Elevator.Goal.STOW, ElevatorArm.Goal.AUTO_L4, IntakeArm.Goal.AUTO_L4, GroundIntakeArm.Goal.STOW),
        AUTO_L4(Elevator.Goal.AUTO_L4, ElevatorArm.Goal.AUTO_L4, IntakeArm.Goal.AUTO_L4, GroundIntakeArm.Goal.STOW),

        NET(Elevator.Goal.NET, ElevatorArm.Goal.UPRIGHT, IntakeArm.Goal.NET, GroundIntakeArm.Goal.STOW),
        ALIGN_NET(Elevator.Goal.STOW, ElevatorArm.Goal.UPRIGHT, IntakeArm.Goal.NET, GroundIntakeArm.Goal.STOW),
        FLING_NET(Elevator.Goal.NET, ElevatorArm.Goal.UPRIGHT, IntakeArm.Goal.ALGAE_FLING, GroundIntakeArm.Goal.STOW),

        SAFE(Elevator.Goal.L3, ElevatorArm.Goal.L4, IntakeArm.Goal.STOW, GroundIntakeArm.Goal.STOW);

        private static final Map<Goal, Translation2d> GoalTranslations = new HashMap<>();

        static {
            for (final Goal goal : Goal.values()) {
                GoalTranslations.put(goal, Superstructure.getElevatorExtensionTranslation(goal));
            }
        }

        public final Elevator.Goal elevatorGoal;
        public final ElevatorArm.Goal elevatorArmGoal;
        public final IntakeArm.Goal intakeArmGoal;
        public final GroundIntakeArm.Goal groundIntakeArmGoal;

        Goal(
                final Elevator.Goal elevatorGoal,
                final ElevatorArm.Goal elevatorArmGoal,
                final IntakeArm.Goal intakeArmGoal,
                final GroundIntakeArm.Goal groundIntakeArmGoal
        ) {
            this.elevatorGoal = elevatorGoal;
            this.elevatorArmGoal = elevatorArmGoal;
            this.intakeArmGoal = intakeArmGoal;
            this.groundIntakeArmGoal = groundIntakeArmGoal;
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

    private enum CollisionDetectionOrder {
        PIVOT_UP_MOVE_ELEVATOR_ARM_FIRST,
        PIVOT_DOWN_MOVE_GROUND_INTAKE_FIRST;

        public static CollisionDetectionOrder select(
                final double currentAngleRots,
                final double desiredAngleRots
        ) {
            return desiredAngleRots >= currentAngleRots
                    ? CollisionDetectionOrder.PIVOT_UP_MOVE_ELEVATOR_ARM_FIRST
                    : CollisionDetectionOrder.PIVOT_DOWN_MOVE_GROUND_INTAKE_FIRST;
        }
    }

    private enum CollisionAvoidanceOrder {
        MOVE_BOTH,
        MOVE_NOTHING,
        MOVE_ONLY_ELEVATOR_ARM,
        MOVE_ELEVATOR_ARM_FIRST,
        MOVE_GROUND_INTAKE_FIRST
    }

    protected static final String LogKey = "Superstructure";
    public static final double AllowableExtensionForDrivingMeters =
            Goal.GoalTranslations.get(Goal.SAFE).getNorm();

    public static final Translation2d GroundIntakeArmCollisionZoneSize = new Translation2d(
            Units.inchesToMeters(17.561254 + 5),
            Units.inchesToMeters(9.77 + 2.5)
    );

    private final Elevator elevator;
    private final ElevatorArm elevatorArm;
    private final IntakeArm intakeArm;
    private final GroundIntakeArm groundIntakeArm;

    private Goal desiredGoal = Goal.STOW;
    private Goal runningGoal = desiredGoal;
    private Goal atGoal = desiredGoal;

    private final MutableEllipse2d groundIntakeCurrentCollisionZone;
    private final MutableEllipse2d groundIntakeDesiredCollisionZone;

    private final EventLoop eventLoop;

    private final LoggedTrigger.Group group;
    private final LoggedTrigger desiredGoalIsRunningGoal;
    private final LoggedTrigger desiredGoalIsAtGoal;
    private final LoggedTrigger desiredGoalIsDynamic;

    private final LoggedTrigger allowedToChangeGoal;

    private final LoggedTrigger desiresUpwardsMotion;
    private final LoggedTrigger desiresDownwardsMotion;
    private final LoggedTrigger desiredGoalChanged;

    private final LoggedTrigger desiredGoalNotStow;
    private final LoggedTrigger atSuperstructureSetpoint;

    public final LoggedTrigger unsafeToDrive;

    public Superstructure(
            final ElevatorArm elevatorArm,
            final Elevator elevator,
            final IntakeArm intakeArm,
            final GroundIntakeArm groundIntakeArm
    ) {
        this.elevatorArm = elevatorArm;
        this.elevator = elevator;
        this.intakeArm = intakeArm;
        this.groundIntakeArm = groundIntakeArm;

        final Translation2d groundIntakeArmBoundingBoxSize = groundIntakeArm.getBoundingBoxSize();
        this.groundIntakeCurrentCollisionZone = new MutableEllipse2d(
                getGroundIntakeArmCenterPose(),
                Math.max(
                        GroundIntakeArmCollisionZoneSize.getX(),
                        groundIntakeArmBoundingBoxSize.getX()
                ) / 2,
                Math.max(
                        GroundIntakeArmCollisionZoneSize.getY(),
                        groundIntakeArmBoundingBoxSize.getY()
                ) / 2
        );
        this.groundIntakeDesiredCollisionZone = new MutableEllipse2d(
                getGroundIntakeArmCenterPose(desiredGoal),
                groundIntakeCurrentCollisionZone.getXSemiAxis(),
                groundIntakeCurrentCollisionZone.getYSemiAxis()
        );

        this.eventLoop = new EventLoop();

        this.group = LoggedTrigger.Group.from(LogKey, eventLoop);
        this.desiredGoalIsRunningGoal = group.t("desiredGoalIsRunningGoal", () -> desiredGoal == runningGoal);
        this.desiredGoalChanged = group.t("desiredGoalChanged", () -> desiredGoal != runningGoal);
        this.desiredGoalIsAtGoal = group.t("desiredGoalIsAtGoal", () -> desiredGoal == atGoal);
        this.desiredGoalIsDynamic = group.t("desiredGoalIsDynamic", () -> desiredGoal == Goal.DYNAMIC);
        this.desiredGoalNotStow = group.t("desiredGoalNotStow", () -> desiredGoal != Goal.STOW);
        this.atSuperstructureSetpoint = elevator.atSetpoint
                .and(elevatorArm.atSetpoint)
                .and(intakeArm.atSetpoint)
                .and(desiredGoalIsAtGoal);

        this.unsafeToDrive = extendedBeyond(AllowableExtensionForDrivingMeters);

        this.allowedToChangeGoal = desiredGoalIsDynamic.negate()
                .and((desiredGoalIsAtGoal.and(atSuperstructureSetpoint)).negate());
        this.desiresUpwardsMotion = group.t("desiresUpwardsMotion", () -> {
            final Translation2d currentTranslation = getElevatorExtensionTranslation();
            final Translation2d desiredTranslation = Goal.GoalTranslations.get(desiredGoal);

            return desiredTranslation.getY() >= currentTranslation.getY();
        });
        this.desiresDownwardsMotion = desiresUpwardsMotion.negate();

        final Command upwardsGoalChange = upwardsGoalChange();
        desiredGoalChanged.and(allowedToChangeGoal).and(desiresUpwardsMotion)
                .onTrue(Commands.runOnce(() -> {
                    upwardsGoalChange.cancel();
                    upwardsGoalChange.schedule();
                }).withName("ScheduleUpwardsGoalChange"));

        final Command downwardsGoalChange = downwardsGoalChange();
        desiredGoalChanged.and(allowedToChangeGoal).and(desiresDownwardsMotion)
                .onTrue(Commands.runOnce(() -> {
                    downwardsGoalChange.cancel();
                    downwardsGoalChange.schedule();
                }).withName("ScheduleDownwardsGoalChange"));

        elevatorArm.setGoal(desiredGoal.elevatorArmGoal);
        elevator.setGoal(desiredGoal.elevatorGoal);
        intakeArm.setGoal(desiredGoal.intakeArmGoal);
    }

    private Command upwardsGoalChange() {
        final Container<CollisionDetectionOrder> detectionOrderContainer = Container.empty();
        final Container<CollisionAvoidanceOrder> avoidanceOrderContainer = Container.empty();
        return FastCommands.sequence(
                Commands.runOnce(() -> {
                    this.atGoal = Goal.NONE;
                    this.runningGoal = desiredGoal;

                    final CollisionDetectionOrder detectionOrder = CollisionDetectionOrder.select(
                            elevatorArm.getPivotPosition().getRotations(),
                            runningGoal.elevatorArmGoal.getPivotPositionGoalRots()
                    );
                    final CollisionAvoidanceOrder avoidanceOrder =
                            getCollisionAvoidanceStrategy(detectionOrder);

                    Logger.recordOutput(LogKey + "/CollisionDetectionOrder", detectionOrder);
                    Logger.recordOutput(LogKey + "/CollisionAvoidanceOrder", avoidanceOrder);

                    detectionOrderContainer.set(detectionOrder);
                    avoidanceOrderContainer.set(avoidanceOrder);

                    switch (avoidanceOrder) {
                        case MOVE_BOTH -> {
                            elevator.setGoal(Elevator.Goal.STOP);

                            elevatorArm.setGoal(runningGoal.elevatorArmGoal);
                            intakeArm.setGoal(runningGoal.intakeArmGoal);
                            groundIntakeArm.setGoal(runningGoal.groundIntakeArmGoal);
                        }
                        case MOVE_ONLY_ELEVATOR_ARM, MOVE_ELEVATOR_ARM_FIRST -> {
                            elevator.setGoal(Elevator.Goal.STOP);
                            groundIntakeArm.setGoal(GroundIntakeArm.Goal.STOP);

                            elevatorArm.setGoal(runningGoal.elevatorArmGoal);
                            intakeArm.setGoal(runningGoal.intakeArmGoal);
                        }
                        case MOVE_GROUND_INTAKE_FIRST -> {
                            elevatorArm.setGoal(ElevatorArm.Goal.STOP);
                            elevator.setGoal(Elevator.Goal.STOP);
                            intakeArm.setGoal(IntakeArm.Goal.STOP);

                            groundIntakeArm.setGoal(runningGoal.groundIntakeArmGoal);
                        }
                        case MOVE_NOTHING -> {
                            elevatorArm.setGoal(ElevatorArm.Goal.STOP);
                            elevator.setGoal(Elevator.Goal.STOP);
                            intakeArm.setGoal(IntakeArm.Goal.STOP);
                            groundIntakeArm.setGoal(GroundIntakeArm.Goal.STOP);
                        }
                    }
                }),

                Commands.waitUntil(() -> switch (avoidanceOrderContainer.get()) {
                    case MOVE_BOTH, MOVE_ONLY_ELEVATOR_ARM, MOVE_ELEVATOR_ARM_FIRST ->
                            elevatorArm.atGoal(runningGoal.elevatorArmGoal)
                                && intakeArm.atGoal(runningGoal.intakeArmGoal);
                    case MOVE_GROUND_INTAKE_FIRST -> groundIntakeArm.atGoal(runningGoal.groundIntakeArmGoal);
                    case MOVE_NOTHING -> true;
                }).withTimeout(4),

                Commands.runOnce(() -> {
                    final CollisionAvoidanceOrder currentAvoidanceOrder =
                            getCollisionAvoidanceStrategy(detectionOrderContainer.get());
                    Logger.recordOutput(LogKey + "/CollisionAvoidanceOrder", currentAvoidanceOrder);

                    if (currentAvoidanceOrder == CollisionAvoidanceOrder.MOVE_BOTH) {
                        final CollisionAvoidanceOrder originalAvoidanceOrder = avoidanceOrderContainer.get();
                        switch (originalAvoidanceOrder) {
                            case MOVE_BOTH, MOVE_ONLY_ELEVATOR_ARM, MOVE_ELEVATOR_ARM_FIRST -> {
                                groundIntakeArm.setGoal(runningGoal.groundIntakeArmGoal);
                                elevator.setGoal(runningGoal.elevatorGoal);
                            }
                            case MOVE_GROUND_INTAKE_FIRST -> {
                                elevatorArm.setGoal(runningGoal.elevatorArmGoal);
                                intakeArm.setGoal(runningGoal.intakeArmGoal);
                            }
                            case MOVE_NOTHING -> {}
                        }
                    }
                }),

                FastCommands.sequence(
                        Commands.waitUntil(() ->
                                elevatorArm.atGoal(runningGoal.elevatorArmGoal)
                                        && intakeArm.atGoal(runningGoal.intakeArmGoal)),
                        Commands.runOnce(() -> elevator.setGoal(runningGoal.elevatorGoal))
                )
                        .onlyIf(() -> switch (avoidanceOrderContainer.get()) {
                            case MOVE_GROUND_INTAKE_FIRST -> true;
                            case MOVE_BOTH, MOVE_ONLY_ELEVATOR_ARM, MOVE_ELEVATOR_ARM_FIRST, MOVE_NOTHING -> false;
                        })
                        .withTimeout(4),

                FastCommands.sequence(
                        Commands.waitUntil(
                                elevatorArm.atSetpoint
                                        .and(elevator.atSetpoint)
                                        .and(intakeArm.atSetpoint)
                                        .and(groundIntakeArm.atSetpoint)
                        ),
                        Commands.runOnce(() -> this.atGoal = runningGoal)
                ).withTimeout(4)
        )
                .onlyWhile(() -> desiredGoal == runningGoal)
                .withName("UpwardsGoalChange");
    }

    private Command downwardsGoalChange() {
        final Container<CollisionDetectionOrder> detectionOrderContainer = Container.empty();
        final Container<CollisionAvoidanceOrder> avoidanceOrderContainer = Container.empty();
        return FastCommands.sequence(
                Commands.runOnce(() -> {
                    this.atGoal = Goal.NONE;
                    this.runningGoal = desiredGoal;

                    final CollisionDetectionOrder detectionOrder = CollisionDetectionOrder.select(
                            elevatorArm.getPivotPosition().getRotations(),
                            runningGoal.elevatorArmGoal.getPivotPositionGoalRots()
                    );
                    final CollisionAvoidanceOrder avoidanceOrder =
                            getCollisionAvoidanceStrategy(detectionOrder);

                    Logger.recordOutput(LogKey + "/CollisionDetectionOrder", detectionOrder);
                    Logger.recordOutput(LogKey + "/CollisionAvoidanceOrder", avoidanceOrder);

                    detectionOrderContainer.set(detectionOrder);
                    avoidanceOrderContainer.set(avoidanceOrder);

                    switch (avoidanceOrder) {
                        case MOVE_BOTH, MOVE_GROUND_INTAKE_FIRST -> {
                            elevatorArm.setGoal(ElevatorArm.Goal.STOP);

                            elevator.setGoal(runningGoal.elevatorGoal);
                            intakeArm.setGoal(runningGoal.intakeArmGoal);
                            groundIntakeArm.setGoal(runningGoal.groundIntakeArmGoal);
                        }
                        case MOVE_ONLY_ELEVATOR_ARM, MOVE_ELEVATOR_ARM_FIRST -> {
                            elevatorArm.setGoal(ElevatorArm.Goal.STOP);
                            groundIntakeArm.setGoal(GroundIntakeArm.Goal.STOP);

                            elevator.setGoal(runningGoal.elevatorGoal);
                            intakeArm.setGoal(runningGoal.intakeArmGoal);
                        }
                        case MOVE_NOTHING -> {
                            elevatorArm.setGoal(ElevatorArm.Goal.STOP);
                            elevator.setGoal(Elevator.Goal.STOP);
                            intakeArm.setGoal(IntakeArm.Goal.STOP);
                            groundIntakeArm.setGoal(GroundIntakeArm.Goal.STOP);
                        }
                    }
                }),

                Commands.waitUntil(() -> switch (avoidanceOrderContainer.get()) {
                    case MOVE_BOTH, MOVE_ONLY_ELEVATOR_ARM, MOVE_ELEVATOR_ARM_FIRST ->
                            elevator.atGoal(runningGoal.elevatorGoal)
                                    && intakeArm.atGoal(runningGoal.intakeArmGoal);
                    case MOVE_GROUND_INTAKE_FIRST ->
                            elevator.atGoal(runningGoal.elevatorGoal)
                                    && intakeArm.atGoal(runningGoal.intakeArmGoal)
                                    && groundIntakeArm.atGoal(runningGoal.groundIntakeArmGoal);
                    case MOVE_NOTHING -> true;
                }).withTimeout(4),

                Commands.runOnce(() -> {
                    final CollisionAvoidanceOrder currentAvoidanceOrder =
                            getCollisionAvoidanceStrategy(detectionOrderContainer.get());
                    Logger.recordOutput(LogKey + "/CollisionAvoidanceOrder", currentAvoidanceOrder);

                    if (currentAvoidanceOrder != CollisionAvoidanceOrder.MOVE_GROUND_INTAKE_FIRST) {
                        final CollisionAvoidanceOrder originalAvoidanceOrder = avoidanceOrderContainer.get();
                        switch (originalAvoidanceOrder) {
                            case MOVE_BOTH,
                                 MOVE_ONLY_ELEVATOR_ARM,
                                 MOVE_ELEVATOR_ARM_FIRST,
                                 MOVE_GROUND_INTAKE_FIRST ->
                                    elevatorArm.setGoal(runningGoal.elevatorArmGoal);
                            case MOVE_NOTHING -> {}
                        }
                    }
                }),

                FastCommands.sequence(
                        Commands.waitUntil(() -> elevatorArm.atGoal(runningGoal.elevatorArmGoal)),
                        Commands.runOnce(() -> groundIntakeArm.setGoal(runningGoal.groundIntakeArmGoal))
                )
                        .onlyIf(() -> switch (avoidanceOrderContainer.get()) {
                            case MOVE_ELEVATOR_ARM_FIRST -> true;
                            case MOVE_BOTH, MOVE_ONLY_ELEVATOR_ARM, MOVE_GROUND_INTAKE_FIRST, MOVE_NOTHING -> false;
                        })
                        .withTimeout(4),

                FastCommands.sequence(
                        Commands.waitUntil(
                                elevatorArm.atSetpoint
                                        .and(elevator.atSetpoint)
                                        .and(intakeArm.atSetpoint)
                                        .and(groundIntakeArm.atSetpoint)
                        ),
                        Commands.runOnce(() -> this.atGoal = runningGoal)
                ).withTimeout(4)
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

        final Translation2d collisionLine = getCollisionLine();
        Logger.recordOutput(LogKey + "/CollisionLine", new Pose3d(
                collisionLine.getX(),
                0,
                collisionLine.getY(),
                new Rotation3d(
                        0,
                        collisionLine
                                .getAngle()
                                .unaryMinus()
                                .getRadians(),
                        0
                )
        ));

        Logger.recordOutput(LogKey + "/RunningGoal", runningGoal.toString());
        Logger.recordOutput(LogKey + "/DesiredGoal", desiredGoal.toString());
        Logger.recordOutput(LogKey + "/AtGoal", atGoal.toString());

        Logger.recordOutput(LogKey + "/DesiredGoalNotStow", desiredGoalNotStow);
        Logger.recordOutput(LogKey + "/AtSetpoint", atSuperstructureSetpoint);
        Logger.recordOutput(LogKey + "/UnsafeToDrive", unsafeToDrive);

        Logger.recordOutput(LogKey + "/ExtensionDistanceMeters", getElevatorExtensionTranslation().getNorm());
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
                intakeArm.getPivotPosition(),
                groundIntakeArm.getPivotPosition()
        );
    }

    public LoggedTrigger atSetpoint(final Supplier<Goal> goalSupplier) {
        return atSuperstructureSetpoint.and(group.t("atGoal", () -> atGoal == goalSupplier.get()));
    }

    public LoggedTrigger atSetpoint(final Goal goal) {
        return atSetpoint(() -> goal);
    }

    public LoggedTrigger extendedBeyond(final double distance) {
        return group.t("extendedBeyond", () -> getElevatorExtensionTranslation().getNorm() > distance);
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

    @SuppressWarnings("unused")
    public Set<Subsystem> getRequirements() {
        return Set.of(elevator, elevatorArm, intakeArm);
    }

    private Translation2d getElevatorExtensionTranslation() {
        return SuperstructureSolver.getElevatorExtensionTranslation(
                elevator.getExtensionMeters(),
                elevatorArm.getPivotPosition()
        );
    }

    private static Translation2d getElevatorExtensionTranslation(final Goal goal) {
        return SuperstructureSolver.getElevatorExtensionTranslation(
                goal.elevatorGoal.getPositionGoalMeters(),
                Rotation2d.fromRotations(goal.elevatorArmGoal.getPivotPositionGoalRots())
        );
    }

    private Translation2d getCollisionLine() {
        final Rotation2d elevatorArmPosition = elevatorArm.getPivotPosition();
        final Translation2d elevatorExtensionLine = SuperstructureSolver
                .getElevatorArmPivotOrigin2d()
                .plus(SuperstructureSolver.getElevatorBaseStageTranslation(elevatorArmPosition))
                .plus(getElevatorExtensionTranslation());

        return SuperstructureSolver.getLowestPointOnGroundIntake(
                elevatorArmPosition,
                intakeArm.getPivotPosition(),
                elevatorExtensionLine
        );
    }

    private Translation2d getCollisionLine(final Goal goal) {
        final Rotation2d elevatorArmPosition =
                Rotation2d.fromRotations(goal.elevatorArmGoal.getPivotPositionGoalRots());
        final Translation2d elevatorExtensionLine = SuperstructureSolver
                .getElevatorArmPivotOrigin2d()
                .plus(SuperstructureSolver.getElevatorBaseStageTranslation(elevatorArmPosition))
                .plus(Superstructure.getElevatorExtensionTranslation(goal));

        return SuperstructureSolver.getLowestPointOnGroundIntake(
                elevatorArmPosition,
                Rotation2d.fromRotations(goal.intakeArmGoal.getPivotPositionGoalRots()),
                elevatorExtensionLine
        );
    }

    private Pose2d getGroundIntakeArmCenterPose(
            final Rotation2d pivotPosition,
            final Translation2d boundingBoxSize
    ) {
        final Pose2d groundIntakePivotPose = SuperstructureSolver
                .getGroundIntakePivotPose2d(pivotPosition);

        final Rotation2d pivotAngle = groundIntakePivotPose.getRotation();
        final double halfLength = boundingBoxSize.getX() / 2;
        return new Pose2d(
                groundIntakePivotPose.getX() + (halfLength * pivotAngle.getCos()),
                groundIntakePivotPose.getY() + (halfLength * pivotAngle.getSin()),
                pivotAngle
        );
    }

    private Pose2d getGroundIntakeArmCenterPose() {
        return getGroundIntakeArmCenterPose(groundIntakeArm.getPivotPosition(), groundIntakeArm.getBoundingBoxSize());
    }

    private Pose2d getGroundIntakeArmCenterPose(final Goal goal) {
        return getGroundIntakeArmCenterPose(
                Rotation2d.fromRotations(goal.groundIntakeArmGoal.getPivotPositionGoalRots()),
                groundIntakeArm.getBoundingBoxSize()
        );
    }

    @SuppressWarnings("unused")
    public Optional<Goal> getClosestGoal(final Set<Goal> goalWhitelist) {
        final Translation2d currentTranslation = getElevatorExtensionTranslation();

        Goal closestGoal = null;
        double minDistance = Double.MAX_VALUE;
        for (final Map.Entry<Goal, Translation2d> goalTranslationEntry
                : Goal.GoalTranslations.entrySet()
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

    private CollisionAvoidanceOrder getCollisionAvoidanceStrategy(final CollisionDetectionOrder order) {
        final Translation2d elevatorArmOrigin = SuperstructureSolver.getElevatorArmPivotOrigin2d();
        final Translation2d desiredCollisionLine = getCollisionLine(desiredGoal);
        final Translation2d checkCollisionLine = switch(order) {
            case PIVOT_UP_MOVE_ELEVATOR_ARM_FIRST -> getCollisionLine();
            case PIVOT_DOWN_MOVE_GROUND_INTAKE_FIRST -> desiredCollisionLine;
        };

        // TODO move/duplicate to periodic for logging
        groundIntakeCurrentCollisionZone.setCenter(getGroundIntakeArmCenterPose());
        groundIntakeDesiredCollisionZone.setCenter(getGroundIntakeArmCenterPose(desiredGoal));

        final boolean illegalDesiredState = checkLineEllipseIntersection(
                elevatorArmOrigin,
                desiredCollisionLine,
                groundIntakeDesiredCollisionZone
        );

        if (illegalDesiredState) {
            final boolean canStillMoveElevatorArm = checkLineEllipseIntersection(
                    elevatorArmOrigin,
                    desiredCollisionLine,
                    groundIntakeCurrentCollisionZone
            );

            return canStillMoveElevatorArm
                    ? CollisionAvoidanceOrder.MOVE_ONLY_ELEVATOR_ARM
                    : CollisionAvoidanceOrder.MOVE_NOTHING;
        }

        final boolean waitUntilSafe = checkLineEllipseIntersection(
                elevatorArmOrigin,
                checkCollisionLine,
                switch (order) {
                    case PIVOT_UP_MOVE_ELEVATOR_ARM_FIRST -> groundIntakeDesiredCollisionZone;
                    case PIVOT_DOWN_MOVE_GROUND_INTAKE_FIRST -> groundIntakeCurrentCollisionZone;
                }
        );

        if (waitUntilSafe) {
            return switch (order) {
                case PIVOT_UP_MOVE_ELEVATOR_ARM_FIRST -> CollisionAvoidanceOrder.MOVE_ELEVATOR_ARM_FIRST;
                case PIVOT_DOWN_MOVE_GROUND_INTAKE_FIRST -> CollisionAvoidanceOrder.MOVE_GROUND_INTAKE_FIRST;
            };
        } else {
            return CollisionAvoidanceOrder.MOVE_BOTH;
        }
    }

    private static boolean checkLineEllipseIntersection(
            final Translation2d origin,
            final Translation2d line,
            final MutableEllipse2d ellipse
    ) {
        final double x0 = origin.getX();
        final double y0 = origin.getY();
        final double dx = line.getX() - x0;
        final double dy = line.getY() - y0;

        final Pose2d center = ellipse.getCenter();
        final double h = center.getX();
        final double k = center.getY();
        final Rotation2d theta = center.getRotation();

        final double a = ellipse.getXSemiAxis();
        final double b = ellipse.getYSemiAxis();

        final double cos = theta.getCos();
        final double sin = theta.getSin();

        final double dxCos = dx * cos + dy * sin;
        final double dxSin = dx * sin - dy * cos;
        final double x0hCos = (x0 - h) * cos + (y0 - k) * sin;
        final double x0hSin = (x0 - h) * sin - (y0 - k) * cos;

        final double aSquared = a * a;
        final double bSquared = b * b;

        final double A = ((dxCos * dxCos) / aSquared) + ((dxSin * dxSin) / bSquared);
        final double B = 2 * (((x0hCos * dxCos) / aSquared) + ((x0hSin * dxSin) / bSquared));
        final double C = ((x0hCos * x0hCos) / aSquared + (x0hSin * x0hSin) / bSquared) - 1;

        final double discriminant = (B * B) - (4 * A * C);

        return discriminant >= 0;

//        if (discriminant < 0) {
//            return new Translation2d[0];
//        }
//
//        final double sqrtDiscriminant = Math.sqrt(discriminant);
//        final double t1 = (-B + sqrtDiscriminant) / (2 * A);
//        final double t2 = (-B - sqrtDiscriminant) / (2 * A);
//
//        final boolean t1Exists = t1 >= 0 && t1 <= 1;
//        final boolean t2Exists = discriminant > 0 && t2 >= 0 && t2 <= 1;
//
//        if (t1Exists && t2Exists) {
//            return new Translation2d[] {
//                    new Translation2d(x0 + t1 * dx, y0 + t1 * dx),
//                    new Translation2d(x0 + t2 * dx, y0 + t2 * dx)
//            };
//        } else if (t1Exists) {
//            return new Translation2d[] {new Translation2d(x0 + t1 * dx, y0 + t1 * dx)};
//        } else {
//            return new Translation2d[0];
//        }
    }
}