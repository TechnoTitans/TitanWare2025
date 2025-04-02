package frc.robot;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.FieldConstants.Reef;
import frc.robot.state.GamepieceState;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.drive.controllers.HolonomicDriveController;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.distal.IntakeArm;
import frc.robot.utils.Container;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static edu.wpi.first.wpilibj2.command.Commands.*;

public class ScoreCommands {
    public enum CoralStation {
        LEFT,
        RIGHT;

        public static Pose2d getCoralStation(final CoralStation station) {
            final List<Pose2d> pickupPoses = FieldConstants.getHPPickupPoses();
            return switch (station) {
                case LEFT -> pickupPoses.get(0);
                case RIGHT -> pickupPoses.get(1);
            };
        }
    }

    public enum Level {
        L1(Reef.Level.L1, Superstructure.Goal.L1),
        L2(Reef.Level.L2, Superstructure.Goal.L2),
        L3(Reef.Level.L3, Superstructure.Goal.L3),
        L4(Reef.Level.L4, Superstructure.Goal.L4);

        public static final Map<Reef.Level, Superstructure.Goal> LevelMap = new HashMap<>();

        static {
            for (final Level level : Level.values()) {
                LevelMap.put(level.level, level.goal);
            }
        }

        public final Reef.Level level;
        public final Superstructure.Goal goal;

        Level(
                final Reef.Level level,
                final Superstructure.Goal goal
        ) {
            this.level = level;
            this.goal = goal;
        }
    }

    public record ScorePosition(
            Reef.Side side,
            Level level
    ) {
    }

    private final Swerve swerve;
    private final Superstructure superstructure;
    private final Intake intake;
    private final GamepieceState gamepieceState;

    public ScoreCommands(
            final Swerve swerve,
            final Superstructure superstructure,
            final Intake intake,
            final GamepieceState gamepieceState
    ) {
        this.swerve = swerve;
        this.superstructure = superstructure;
        this.intake = intake;
        this.gamepieceState = gamepieceState;
    }

    @SuppressWarnings("unused")
    public Supplier<ScorePosition> getScorePositionSupplier(final CommandXboxController controller) {
        return () -> {
            final Reef.Side side;
            final Level level;

            if (controller.getRightX() > 0) {
                side = Reef.Side.RIGHT;
            } else {
                side = Reef.Side.LEFT;
            }

            final int povPosition = controller.getHID().getPOV();
            if (povPosition == 0) {
                level = Level.L4;
            } else if (povPosition == 90) {
                level = Level.L3;
            } else if (povPosition == 180) {
                level = Level.L2;
            } else if (povPosition == 270) {
                level = Level.L1;
            } else {
                level = Level.L2;
            }

            return new ScorePosition(side, level);
        };
    }

    public Pose2d offsetScoringPoseWithCoralPosition(final Pose2d targetPose) {
        final Transform2d coralDistanceOffset = new Transform2d(
                0,
                gamepieceState.hasCoral.getAsBoolean()
                        ? intake.coralDistanceIntakeCenterMeters.getAsDouble()
                        : 0,
                Rotation2d.kZero
        );

        return targetPose.transformBy(coralDistanceOffset);
    }

    public Command intakeFacingClosestCoralStation(
            final DoubleSupplier leftStickYInput,
            final DoubleSupplier leftStickXInput
    ) {
        //noinspection SuspiciousNameCombination
        return parallel(
                swerve.teleopFacingAngle(
                        leftStickYInput,
                        leftStickXInput,
                        () -> {
                            final Pose2d currentPose = swerve.getPose();
                            final Pose2d nearestStation = currentPose.nearest(FieldConstants.getHPPickupPoses());

                            return nearestStation.getRotation().rotateBy(Rotation2d.kPi);
                        }
                ).onlyIf(superstructure.unsafeToDrive.negate()),
                superstructure.toGoal(Superstructure.Goal.HP),
                intake.intakeCoralHP()
        );
    }

    public Command scoreAtFixedPosition(final Supplier<ScorePosition> scorePositionSupplier) {
        final Container<ScorePosition> scorePositionContainer = Container.of(scorePositionSupplier.get());
        final Runnable updateScorePosition = () -> scorePositionContainer.value = scorePositionSupplier.get();

        final Trigger shouldUseEarlyAlign = new Trigger(() ->
                switch (scorePositionContainer.value.level) {
                    case L1, L2 -> false;
                    case L3, L4 -> true;
                }
        );

        final Supplier<Map<Reef.Side, Map<Reef.Level, Pose2d>>> scoringPoseMap = () -> {
            final Map<Reef.Face, Map<Reef.Side, Map<Reef.Level, Pose2d>>> branchScoringPositions =
                    FieldConstants.getBranchScoringPositions();
            final Map<Reef.Face, Pose2d> reefCenterFacePoses = FieldConstants.getReefScoringCenterPoses();
            final Translation2d currentTranslation = swerve.getPose().getTranslation();

            double closestDistanceMeters = Double.MAX_VALUE;
            Map<Reef.Side, Map<Reef.Level, Pose2d>> poseMap = null;
            for (final Reef.Face face : Reef.Face.values()) {
                final Pose2d centerFace = reefCenterFacePoses.get(face);

                final double distanceMeters = centerFace.getTranslation()
                        .getDistance(currentTranslation);

                if (distanceMeters < closestDistanceMeters) {
                    poseMap = branchScoringPositions.get(face);
                    closestDistanceMeters = distanceMeters;
                }
            }
            return poseMap;
        };

        final Container<Map<Reef.Side, Map<Reef.Level, Pose2d>>> scoringPoseMapContainer = Container.empty();
        final Runnable updateScoringPoseMap = () -> scoringPoseMapContainer.value = scoringPoseMap.get();

        final Supplier<Pose2d> scoringPoseSupplier = () -> {
            final Pose2d scoringPose = scoringPoseMapContainer.value
                    .get(scorePositionContainer.value.side)
                    .get(scorePositionContainer.value.level.level);

            return offsetScoringPoseWithCoralPosition(scoringPose);
        };

        final Supplier<Pose2d> reefAlignmentPoseSupplier = () -> scoringPoseSupplier
                .get()
                .transformBy(FieldConstants.ALIGN_DISTANCE_OFFSET);

        final Trigger atAlignReef = swerve.atPoseTrigger(reefAlignmentPoseSupplier);
        final Trigger atReef = swerve.atPoseTrigger(scoringPoseSupplier);

        final Trigger stoppedNearReef = atReef.negate()
                .and(swerve.atPoseAndStoppedTrigger(
                        scoringPoseSupplier,
                        new HolonomicDriveController.PositionTolerance(
                                0.4,
                                Rotation2d.fromDegrees(8)
                        ),
                        new HolonomicDriveController.VelocityTolerance(
                                0.1,
                                Math.PI / 8
                        )
                ))
                .debounce(0.15, Debouncer.DebounceType.kFalling);

        final Container<Superstructure.Goal> superstructureGoalContainer = Container.empty();
        final Runnable setSuperstructureGoalToAlign = () -> superstructureGoalContainer.value
                = Superstructure.Goal.getAlignGoal(scorePositionContainer.value.level.goal);
        final Runnable setSuperstructureGoalToScore = () -> superstructureGoalContainer.value
                = scorePositionContainer.value.level.goal;

        return sequence(
                runOnce(updateScorePosition),
                runOnce(updateScoringPoseMap),
                either(
                        runOnce(setSuperstructureGoalToAlign),
                        runOnce(setSuperstructureGoalToScore),
                        shouldUseEarlyAlign
                ),
                deadline(
                        sequence(
                                superstructure.toInstantGoal(Superstructure.Goal.STOW)
                                        .onlyIf(superstructure.unsafeToDrive),
                                waitUntil(atAlignReef)
                                        .onlyIf(shouldUseEarlyAlign.and(atReef.negate())),
                                deadline(
                                        sequence(
                                                waitUntil(atReef.or(stoppedNearReef)),
                                                runOnce(setSuperstructureGoalToScore),
                                                either(
                                                        waitUntil(superstructure.atDynamicSetpoint),
                                                        waitUntil(superstructure
                                                                .atSetpoint(superstructureGoalContainer))
                                                                .withTimeout(2),
                                                        stoppedNearReef
                                                ),
                                                waitSeconds(10),
                                                intake.scoreCoral()
                                        ),
                                        repeatingSequence(
                                                either(
                                                        superstructure.toDynamic(
                                                                () -> superstructureGoalContainer
                                                                        .value
                                                                        .elevatorArmGoal
                                                                        .getPivotPositionGoalRots(),
                                                                () -> superstructureGoalContainer
                                                                        .value
                                                                        .elevatorGoal
                                                                        .getPositionGoalMeters(),
                                                                () -> {
                                                                    final IntakeArm.Goal goal =
                                                                            superstructureGoalContainer
                                                                                    .value
                                                                                    .intakeArmGoal;

                                                                    return goal.getPivotPositionGoalRots();
                                                                }
                                                        ).onlyWhile(stoppedNearReef),
                                                        superstructure.toGoal(superstructureGoalContainer)
                                                                .onlyWhile(stoppedNearReef.negate()),
                                                        stoppedNearReef
                                                )
                                        )
                                )
                        ),
                        either(
                                sequence(
                                        swerve.driveToPose(reefAlignmentPoseSupplier)
                                                .onlyIf(atReef.negate())
                                                .onlyWhile(shouldUseEarlyAlign),
                                        race(
                                                swerve.runToPose(() -> scoringPoseSupplier.get().transformBy(new Transform2d(-0.25, 0, Rotation2d.kZero)))
                                                        .until(atReef),
//                                                swerve.runToPose(scoringPoseSupplier)
//                                                        .until(atReef),
                                                waitSeconds(0.5)
                                                        .andThen(waitUntil(stoppedNearReef))
                                        ),
                                        swerve.runWheelXCommand()
                                ),
                                swerve.runToPose(scoringPoseSupplier)
                                        .onlyWhile(shouldUseEarlyAlign.negate())
                                        .andThen(swerve.runWheelXCommand()),
                                shouldUseEarlyAlign
                        )
                ),
                waitUntil(superstructure.unsafeToDrive.negate())
                        .withTimeout(0.8)
        ).withName("ScoreAtFixedPosition");
    }

    @SuppressWarnings("unused")
    public Command readyScoreAtPosition(final Supplier<ScorePosition> wantScorePosition) {
        final Container<ScorePosition> driveToScorePosition = Container.of(wantScorePosition.get());
        final Consumer<ScorePosition> setDriveToScorePosition =
                (scorePosition) -> driveToScorePosition.value = scorePosition;

        final Trigger shouldUseEarlyAlign = new Trigger(() ->
                switch (wantScorePosition.get().level) {
                    case L1, L2 -> false;
                    case L3, L4 -> true;
                }
        );

        final Supplier<Map<Reef.Side, Map<Reef.Level, Pose2d>>> scoringPoseMap = () -> {
            final Map<Reef.Face, Map<Reef.Side, Map<Reef.Level, Pose2d>>> branchScoringPositions =
                    FieldConstants.getBranchScoringPositions();
            final Map<Reef.Face, Pose2d> reefCenterFacePoses = FieldConstants.getReefScoringCenterPoses();
            final Translation2d currentTranslation = swerve.getPose().getTranslation();

            double closestDistanceMeters = Double.MAX_VALUE;
            Map<Reef.Side, Map<Reef.Level, Pose2d>> poseMap = null;
            for (final Reef.Face face : Reef.Face.values()) {
                final Pose2d centerFace = reefCenterFacePoses.get(face);

                final double distanceMeters = centerFace.getTranslation()
                        .getDistance(currentTranslation);

                if (distanceMeters < closestDistanceMeters) {
                    poseMap = branchScoringPositions.get(face);
                    closestDistanceMeters = distanceMeters;
                }
            }
            return poseMap;
        };

        final Container<Map<Reef.Side, Map<Reef.Level, Pose2d>>> scoringPoseMapContainer = Container.empty();
        final Runnable updateScoringPoseMap = () -> scoringPoseMapContainer.value = scoringPoseMap.get();

        final Supplier<Pose2d> scoringPoseSupplier = () -> {
            final ScorePosition scorePosition = driveToScorePosition.get();
            final Pose2d scoringPose = scoringPoseMapContainer.value
                    .get(scorePosition.side)
                    .get(scorePosition.level.level);

            return offsetScoringPoseWithCoralPosition(scoringPose);
        };

        final Trigger atReef = swerve.atPoseTrigger(scoringPoseSupplier);

        return runOnce(updateScoringPoseMap).andThen(parallel(
                sequence(
                        waitUntil(superstructure.unsafeToDrive.negate()),
                        repeatingSequence(
                                either(
                                        sequence(
                                                swerve.driveToPose(
                                                                () -> scoringPoseSupplier
                                                                        .get()
                                                                        .transformBy(FieldConstants.ALIGN_DISTANCE_OFFSET))
                                                        .onlyIf(atReef.negate())
                                                        .onlyWhile(shouldUseEarlyAlign),
                                                swerve.runToPose(scoringPoseSupplier)
                                        ),
                                        swerve.runToPose(scoringPoseSupplier)
                                                .onlyWhile(shouldUseEarlyAlign.negate()),
                                        shouldUseEarlyAlign
                                )
                        )
                ),
                sequence(
                        superstructure.toInstantGoal(Superstructure.Goal.STOW)
                                .onlyIf(superstructure.unsafeToDrive),
                        deadline(
                                waitUntil(swerve.atHolonomicDrivePose),
                                run(() -> setDriveToScorePosition.accept(wantScorePosition.get()))
                        ),
                        superstructure.runGoal(() ->
                                        Superstructure.Goal.getAlignGoal(wantScorePosition.get().level.goal))
                                .until(swerve.atHolonomicDrivePose)
                                .onlyIf(shouldUseEarlyAlign),
                        repeatingSequence(
                                defer(() -> {
                                    final ScorePosition scorePosition = wantScorePosition.get();
                                    final BooleanSupplier desiredScorePositionHasNotChanged =
                                            () -> scorePosition.equals(wantScorePosition.get());

                                    return sequence(
                                            superstructure.runGoal(Superstructure.Goal.SAFE)
                                                    .until(superstructure.atSetpoint(scorePosition.level.goal)
                                                            .and(superstructure.unsafeToDrive.negate()))
                                                    .onlyIf(superstructure.unsafeToDrive.and(
                                                            // unsafe to drive AND needs to drive
                                                            () -> driveToScorePosition.get().side
                                                                    != scorePosition.side
                                                    )),
                                            runOnce(() -> setDriveToScorePosition.accept(scorePosition)),
                                            waitUntil(swerve.atHolonomicDrivePose),
                                            superstructure.runGoal(scorePosition.level.goal)
                                    ).onlyWhile(desiredScorePositionHasNotChanged);
                                }, superstructure.getRequirements())
                        )
                )
        )).withName("ReadyScoreAtPosition");
    }

    public Command readyScoreAtPositionNoLineup(final Supplier<ScorePosition> scorePositionSupplier) {
        return superstructure.runGoal(() -> scorePositionSupplier.get().level().goal)
                .withName("ReadyScoreAtPositionNoLineup");
    }

    public Command descoreUpperAlgae() {
        final Supplier<Pose2d> descorePoseSupplier = () -> {
            final Pose2d currentPose = swerve.getPose();

            double closestDistance = Double.MAX_VALUE;
            Pose2d closestPose = new Pose2d();
            for (final Pose2d reefPose : FieldConstants.getReefCenterPoses().values()) {
                final double distance = currentPose.getTranslation().getDistance(reefPose.getTranslation());
                if (distance < closestDistance) {
                    closestDistance = distance;
                    closestPose = reefPose;
                }
            }
            return closestPose.plus(FieldConstants.ALGAE_DESCORE_DISTANCE_OFFSET);
        };

        final Supplier<Pose2d> alignPoseSupplier = () -> descorePoseSupplier.get()
                .transformBy(FieldConstants.ALGAE_ALIGN_DISTANCE_OFFSET);

        final Trigger atAlignReef = swerve.atPoseTrigger(alignPoseSupplier);
        final Trigger atReef = swerve.atPoseTrigger(descorePoseSupplier);

        return deadline(
                sequence(
                        superstructure.toInstantGoal(Superstructure.Goal.STOW)
                                .onlyIf(superstructure.unsafeToDrive),
                        waitUntil(atAlignReef).withTimeout(4),
                        deadline(
                                sequence(
                                        waitUntil(atReef).withTimeout(2),
                                        waitUntil(superstructure.atSetpoint(Superstructure.Goal.UPPER_ALGAE)
                                                .and(gamepieceState.hasAlgae)
                                                .and(atAlignReef)).withTimeout(3)
                                ),
                                superstructure.toGoal(Superstructure.Goal.UPPER_ALGAE)
                        )
                ),
                sequence(
                        swerve.driveToPose(alignPoseSupplier),
                        waitUntil(superstructure.atSetpoint(Superstructure.Goal.UPPER_ALGAE))
                                .withTimeout(2),
                        swerve.runToPose(descorePoseSupplier).until(gamepieceState.hasAlgae),
                        waitSeconds(0.1),
                        swerve.driveToPose(alignPoseSupplier)
                ),
                intake.intakeAlgae().asProxy()
        ).withName("DescoreUpperAlgae");
    }

    public Command descoreLowerAlgae() {
        final Supplier<Pose2d> descorePoseSupplier = () -> {
            final Pose2d currentPose = swerve.getPose();

            double closestDistance = Double.MAX_VALUE;
            Pose2d closestPose = new Pose2d();
            for (final Pose2d reefPose : FieldConstants.getReefCenterPoses().values()) {
                final double distance = currentPose.getTranslation().getDistance(reefPose.getTranslation());
                if (distance < closestDistance) {
                    closestDistance = distance;
                    closestPose = reefPose;
                }
            }
            return closestPose.plus(FieldConstants.ALGAE_DESCORE_DISTANCE_OFFSET);
        };

        final Supplier<Pose2d> alignPoseSupplier = () -> descorePoseSupplier.get()
                .transformBy(FieldConstants.ALGAE_ALIGN_DISTANCE_OFFSET);

        final Trigger atAlignReef = swerve.atPoseTrigger(alignPoseSupplier);
        final Trigger atReef = swerve.atPoseTrigger(descorePoseSupplier);

        return deadline(
                sequence(
                        superstructure.toInstantGoal(Superstructure.Goal.STOW)
                                .onlyIf(superstructure.unsafeToDrive),
                        waitUntil(atAlignReef).withTimeout(4),
                        deadline(
                                sequence(
                                        waitUntil(atReef).withTimeout(2),
                                        waitUntil(superstructure.atSetpoint(Superstructure.Goal.LOWER_ALGAE)
                                                .and(gamepieceState.hasAlgae)
                                                .and(atAlignReef)).withTimeout(3)
                                ),
                                superstructure.toGoal(Superstructure.Goal.LOWER_ALGAE)
                        )
                ),
                sequence(
                        swerve.driveToPose(alignPoseSupplier),
                        waitUntil(superstructure.atSetpoint(Superstructure.Goal.LOWER_ALGAE))
                                .withTimeout(2),
                        swerve.runToPose(descorePoseSupplier).until(gamepieceState.hasAlgae),
                        waitSeconds(0.1),
                        swerve.driveToPose(alignPoseSupplier)
                ),
                intake.intakeAlgae().asProxy()
        ).withName("DescoreLowerAlgae");
    }

    public Command scoreAtPosition(final Supplier<ScorePosition> scorePositionSupplier) {
        final Container<Superstructure.Goal> goalContainer = Container.empty();

        return deadline(
                sequence(
                        goalContainer.set(() -> scorePositionSupplier.get().level.goal),
                        deadline(
                                waitUntil(superstructure.atSetpoint(goalContainer))
                                        .withTimeout(3)
                                        .andThen(intake.scoreCoral()),
                                superstructure.toGoal(goalContainer)
                        ),
                        waitUntil(superstructure.unsafeToDrive.negate()
                                .or(superstructure.atSetpoint(Superstructure.Goal.STOW)))
                ),
                swerve.runWheelXCommand()
        ).withName("ScoreAtPositionTeleop");
    }

    public Command scoreNetFlingFacingBarge() {
        final Container<Superstructure.Goal> superstructureGoal = Container.empty();

        final DoubleSupplier axisTarget = () -> FieldConstants.getScoringBargeCenterCage().getX();
        final DoubleSupplier robotX = () -> swerve.getPose().getX();

        return sequence(
                superstructureGoal.set(Superstructure.Goal.STOW),
                deadline(
                        sequence(
                                parallel(
                                        superstructureGoal.set(Superstructure.Goal.ALIGN_NET),
                                        swerve.driveToAxisFacingAngle(
                                                axisTarget,
                                                Swerve.DriveAxis.X,
                                                () -> Rotation2d.kZero
                                        )
                                ).onlyIf(swerve.atAxisTrigger(axisTarget, robotX).negate()),
                                swerve.wheelXCommand(),
                                superstructureGoal.set(Superstructure.Goal.NET),
                                waitUntil(superstructure.extendedBeyond(0.7)),
                                intake.releaseAlgae(),
                                parallel(
                                        superstructureGoal.set(Superstructure.Goal.FLING_NET),
                                        waitUntil(superstructure.atSetpoint(Superstructure.Goal.FLING_NET))
                                ),
                                waitSeconds(1.5)
                        ),
                        superstructure.toGoal(superstructureGoal)
                )
        ).withName("ScoreNetFlingFacingBarge");
    }

    public Command readyScoreProcessor() {
        return superstructure.runGoal(Superstructure.Goal.PROCESSOR)
                .withName("ReadyScoreProcessor");
    }

    public Command scoreProcessor() {
        return deadline(
                sequence(
                        waitUntil(superstructure.atSetpoint(Superstructure.Goal.PROCESSOR))
                                .withTimeout(2),
                        intake.scoreAlgae(),
                        waitUntil(intake.isCurrentAboveAlgaeThreshold.negate()).withTimeout(2)
                ),
                superstructure.toGoal(Superstructure.Goal.PROCESSOR),
                swerve.runWheelXCommand()
        ).withName("ScoreProcessor");
    }

    public Command intakeLowerAlgae() {
        return parallel(
                superstructure.toGoal(Superstructure.Goal.LOWER_ALGAE),
                intake.intakeAlgae().asProxy()
        ).withName("IntakeLowerAlgae");
    }

    public Command intakeUpperAlgae() {
        return parallel(
                superstructure.toGoal(Superstructure.Goal.UPPER_ALGAE),
                intake.intakeAlgae().asProxy()
        ).withName("IntakeUpperAlgae");
    }

    public Command intakeAlgaeFromGround() {
        return parallel(
                superstructure.toGoal(Superstructure.Goal.ALGAE_GROUND),
                intake.intakeAlgae()
        ).withName("IntakeAlgaeFromGround");
    }

    @SuppressWarnings("SuspiciousNameCombination")
    public Command readyClimb(
            final DoubleSupplier leftStickYInput,
            final DoubleSupplier leftStickXInput
    ) {
        return parallel(
                superstructure.runGoal(Superstructure.Goal.CLIMB),
                swerve.teleopFacingAngle(
                        leftStickYInput,
                        leftStickXInput,
                        () -> Robot.IsRedAlliance.getAsBoolean() ? Rotation2d.kZero : Rotation2d.k180deg
                )
        ).withName("ReadyClimb");
    }
}