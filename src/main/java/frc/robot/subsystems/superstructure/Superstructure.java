package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.superstructure.distal.IntakeArm;
import frc.robot.subsystems.superstructure.elevator.Elevator;
import frc.robot.subsystems.superstructure.ground.GroundIntakeArm;
import frc.robot.subsystems.superstructure.proximal.ElevatorArm;
import frc.robot.utils.geometry.Ellipse2dHelpers;
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
        UPPER_ALGAE(Elevator.Goal.UPPER_ALGAE, ElevatorArm.Goal.UPPER_ALGAE, IntakeArm.Goal.UPPER_ALGAE, GroundIntakeArm.Goal.STOW),
        LOWER_ALGAE(Elevator.Goal.LOWER_ALGAE, ElevatorArm.Goal.LOWER_ALGAE, IntakeArm.Goal.LOWER_ALGAE, GroundIntakeArm.Goal.STOW),

        HP(Elevator.Goal.HP, ElevatorArm.Goal.HP, IntakeArm.Goal.HP, GroundIntakeArm.Goal.STOW),
        GROUND_INTAKE(Elevator.Goal.HANDOFF, ElevatorArm.Goal.HANDOFF, IntakeArm.Goal.HANDOFF, GroundIntakeArm.Goal.INTAKE),
        PROCESSOR(Elevator.Goal.PROCESSOR, ElevatorArm.Goal.PROCESSOR, IntakeArm.Goal.PROCESSOR, GroundIntakeArm.Goal.STOW),

        L1(Elevator.Goal.L1, ElevatorArm.Goal.L1, IntakeArm.Goal.L1, GroundIntakeArm.Goal.STOW),
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

    protected static final String LogKey = "Superstructure";
    public static final double AllowableExtensionForDrivingMeters =
            Goal.GoalTranslations.get(Goal.SAFE).getNorm();

    public static final Translation2d GroundIntakeArmCollisionZoneSize = new Translation2d(
            Units.inchesToMeters(17.561254),
            Units.inchesToMeters(9.77)
    );

    private final Elevator elevator;
    private final ElevatorArm elevatorArm;
    private final IntakeArm intakeArm;
    private final GroundIntakeArm groundIntakeArm;

    private Goal desiredGoal = Goal.STOW;
    private Goal runningGoal = desiredGoal;
    private Goal atGoal = desiredGoal;

    private final Ellipse2d groundIntakeCollisionZone;

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
            final IntakeArm intakeArm,
            final GroundIntakeArm groundIntakeArm
    ) {
        this.elevatorArm = elevatorArm;
        this.elevator = elevator;
        this.intakeArm = intakeArm;
        this.groundIntakeArm = groundIntakeArm;

        final Translation2d groundIntakeArmBoundingBoxSize = groundIntakeArm.getBoundingBoxSize();
        this.groundIntakeCollisionZone = new Ellipse2d(
                getGroundIntakeArmCenterPose(),
                Math.max(GroundIntakeArmCollisionZoneSize.getX(), groundIntakeArmBoundingBoxSize.getX()),
                Math.max(GroundIntakeArmCollisionZoneSize.getY(), groundIntakeArmBoundingBoxSize.getY())
        );

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
            final Translation2d currentTranslation = getElevatorTranslation();
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

                    elevatorArm.setGoal(runningGoal.elevatorArmGoal);
                    intakeArm.setGoal(runningGoal.intakeArmGoal);

                    if (!checkGroundIntakeCollision()) {
                        groundIntakeArm.setGoal(runningGoal.groundIntakeArmGoal);
                    }
                }),

                Commands.waitUntil(elevatorArm.atSetpoint.and(intakeArm.atSetpoint))
                        .withTimeout(4),
                Commands.runOnce(() -> elevator.setGoal(runningGoal.elevatorGoal)),

                Commands.waitUntil(
                        elevatorArm.atSetpoint
                                .and(elevator.atSetpoint)
                                .and(intakeArm.atSetpoint)
                                .and(groundIntakeArm.atSetpoint)
                ).withTimeout(4),
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

                    intakeArm.setGoal(runningGoal.intakeArmGoal);
                    elevator.setGoal(runningGoal.elevatorGoal);
                    groundIntakeArm.setGoal(runningGoal.groundIntakeArmGoal);
                }),

                Commands.waitUntil(
                        elevator.atSetpoint
                                .and(intakeArm.atSetpoint)
                ).withTimeout(4),
                Commands.runOnce(() -> elevatorArm.setGoal(runningGoal.elevatorArmGoal)),

                Commands.waitUntil(
                        elevatorArm.atSetpoint
                                .and(elevator.atSetpoint)
                                .and(intakeArm.atSetpoint)
                                .and(groundIntakeArm.atSetpoint)
                ).withTimeout(4),
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

        Logger.recordOutput(LogKey + "/ExtensionDistanceMeters", getElevatorTranslation().getNorm());
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

    public Trigger atSetpoint(final Supplier<Superstructure.Goal> goalSupplier) {
        return atSuperstructureSetpoint.and(() -> atGoal == goalSupplier.get());
    }

    public Trigger atSetpoint(final Superstructure.Goal goal) {
        return atSetpoint(() -> goal);
    }

    public Trigger extendedBeyond(final double distance) {
        return new Trigger(eventLoop, () -> getElevatorTranslation().getNorm() > distance);
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

    private Translation2d getElevatorTranslation() {
        return new Translation2d(
                elevator.getExtensionMeters(),
                elevatorArm.getPivotPosition()
        );
    }

    private Translation2d getElevatorCollisionLine() {
        return SuperstructureSolver
                .getElevatorArmPivotOrigin2d()
                .plus(getElevatorTranslation());
    }

    private Pose2d getGroundIntakeArmCenterPose() {
        final Rotation2d pivotPosition = groundIntakeArm.getPivotPosition();
        final Pose3d armPose3d = SuperstructureSolver
                .getGroundIntakePose(pivotPosition);

        final Translation2d boundingBoxSize = groundIntakeArm.getBoundingBoxSize();
        final double halfLength = boundingBoxSize.getX() / 2;
        return new Pose2d(
                armPose3d.getX() + (halfLength * pivotPosition.getCos()),
                armPose3d.getZ() + (halfLength * pivotPosition.getSin()),
                pivotPosition
        );
    }

    @SuppressWarnings("unused")
    public Optional<Goal> getClosestGoal(final Set<Goal> goalWhitelist) {
        final Translation2d currentTranslation = getElevatorTranslation();

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

    private static Translation2d getClosestPointOnLineToPoint(
            final Translation2d origin,
            final Translation2d line,
            final Translation2d point
    ) {
        final Translation2d originToPoint = point.minus(origin);
        final Translation2d originToLine = line.minus(origin);
        final double lineX = originToLine.getX();
        final double lineY = originToLine.getY();
        final double magSquared = (lineX * lineX) + (lineY * lineY);
        final double dot = (originToPoint.getX() * lineX)
                + (originToPoint.getY() * lineY);

        final double t = MathUtil.clamp(dot / magSquared, 0, 1);
        if (t <= 0) {
            return origin;
        } else if (t >= 1) {
            return origin.plus(line);
        } else {
            return origin.plus(line.times(t));
        }
    }

    private boolean checkGroundIntakeCollision() {
        final Translation2d elevatorArmOrigin = SuperstructureSolver.getElevatorArmPivotOrigin2d();
        final Translation2d elevatorCollisionLine = getElevatorCollisionLine();

        final Pose2d groundIntakeCollisionZoneCenter = groundIntakeCollisionZone.getCenter();
        final double groundIntakeCollisionZoneXSemiAxis = groundIntakeCollisionZone.getXSemiAxis();
        final double groundIntakeCollisionZoneYSemiAxis = groundIntakeCollisionZone.getYSemiAxis();

        final Translation2d startPoint = getClosestPointOnLineToPoint(
                elevatorArmOrigin,
                elevatorCollisionLine,
                groundIntakeCollisionZoneCenter.getTranslation()
        );

        final int _MAX_ITERATIONS = 50;
        final double distanceToStart = startPoint.getDistance(elevatorArmOrigin);
        final double distanceToEnd = startPoint.getDistance(elevatorCollisionLine);

        final double distancePerIterToStart = distanceToStart / _MAX_ITERATIONS;
        final double distancePerIterToEnd = distanceToEnd / _MAX_ITERATIONS;
        final double distanceIter0 = Math.min(distancePerIterToStart, distancePerIterToEnd);

        final Translation2d offsetPerIter;
        {
            final Rotation2d theta = elevatorCollisionLine
                    .minus(elevatorArmOrigin)
                    .getAngle();
            final Translation2d offset = new Translation2d(distanceIter0, theta);
            final Translation2d toStart = startPoint.minus(offset);
            final Translation2d toEnd = startPoint.plus(offset);

            final double startDistanceToZone = groundIntakeCollisionZone.getDistance(toStart);
            final double endDistanceToZone = groundIntakeCollisionZone.getDistance(toEnd);

            if (MathUtil.isNear(startDistanceToZone, endDistanceToZone, 1e-4)) {
                return Ellipse2dHelpers.contains(
                        groundIntakeCollisionZoneCenter,
                        groundIntakeCollisionZoneXSemiAxis,
                        groundIntakeCollisionZoneYSemiAxis,
                        startPoint
                );
            } else if (startDistanceToZone < endDistanceToZone) {
                offsetPerIter = new Translation2d(-distancePerIterToStart, theta);
            } else if (endDistanceToZone < startDistanceToZone) {
                offsetPerIter = new Translation2d(distancePerIterToEnd, theta);
            } else {
                // should never get here
                return Ellipse2dHelpers.contains(
                        groundIntakeCollisionZoneCenter,
                        groundIntakeCollisionZoneXSemiAxis,
                        groundIntakeCollisionZoneYSemiAxis,
                        startPoint
                );
            }
        }

        final double offsetX = offsetPerIter.getX();
        final double offsetY = offsetPerIter.getY();

        double x = startPoint.getX() + offsetX;
        double y = startPoint.getY() + offsetY;
        for (int i = 0; i < _MAX_ITERATIONS; i++) {
            final boolean contains = Ellipse2dHelpers.contains(
                    groundIntakeCollisionZoneCenter,
                    groundIntakeCollisionZoneXSemiAxis,
                    groundIntakeCollisionZoneYSemiAxis,
                    x,
                    y
            );

            if (contains) {
                return true;
            }

            x += offsetX;
            y += offsetY;
        }

        return false;
    }
}