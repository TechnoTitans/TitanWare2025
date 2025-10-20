package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.FieldConstants.Reef;
import frc.robot.state.GamepieceState;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.drive.controllers.HolonomicDriveController;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.utils.Container;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class ScoreCommands {
    public enum ExtendWhen {
        CLOSE,
        ROTATION_CLOSE,
        ALWAYS
    }

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
        L4(Reef.Level.L4, Superstructure.Goal.L4),
        AUTO_L4(Reef.Level.AUTO_L4, Superstructure.Goal.AUTO_L4);

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
    ) {}

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

    @SuppressWarnings("SameParameterValue")
    private Trigger atRotationTrigger(
            final Supplier<Pose2d> scoringPoseSupplier,
            final Rotation2d rotationTolerance,
            final double rotationVelocityToleranceRadsPerSec
    ) {
        return new Trigger(() -> {
            final Transform2d delta = swerve.getPose().minus(scoringPoseSupplier.get());
            final ChassisSpeeds speeds = swerve.getFieldRelativeSpeeds();

            final double rotationDeltaRads = Math.abs(MathUtil.angleModulus(delta.getRotation().getRadians()));

            return rotationDeltaRads < rotationTolerance.getRadians()
                    && Math.abs(speeds.omegaRadiansPerSecond) < rotationVelocityToleranceRadsPerSec;
        });
    }

    private Supplier<Intake.ScoreMode> scoreModeFromScorePosition(final Supplier<ScorePosition> scorePositionSupplier) {
        return () -> switch (scorePositionSupplier.get().level.level) {
            case AUTO_L4, L4, L3, L2 -> Intake.ScoreMode.RUN_UNTIL_NO_CORAL;
            case L1 -> Intake.ScoreMode.RUN_FOR_TIME;
        };
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
        return Commands.parallel(
                swerve.teleopFacingAngle(
                        leftStickYInput,
                        leftStickXInput,
                        () -> swerve.getPose().nearest(FieldConstants.getHPPickupPoses()).getRotation()
                ).onlyIf(superstructure.unsafeToDrive.negate()),
                superstructure.toGoal(Superstructure.Goal.HP),
                intake.intakeCoralHP()
        ).withName("IntakeFromClosestCoralStation");
    }

    public Command scoreAtFixedPosition(final Supplier<ScorePosition> scorePositionSupplier) {
        final Container<ScorePosition> scorePositionContainer = Container.of(scorePositionSupplier.get());

        final Runnable updateScorePosition = () -> scorePositionContainer.value = scorePositionSupplier.get();
        final Container<ScorePosition> runningScorePositionContainer = Container.empty();


        final Trigger shouldUseEarlyAlign = new Trigger(() ->
                switch (scorePositionContainer.value.level) {
                    case AUTO_L4, L4, L3 -> true;
                    case L2, L1 -> false;
                }
        );
        final Supplier<ExtendWhen> extendWhenSupplier = () ->
                switch (scorePositionContainer.value.level) {
                    case AUTO_L4, L4 -> ExtendWhen.CLOSE;
                    case L3 -> ExtendWhen.ROTATION_CLOSE;
                    case L2, L1 -> ExtendWhen.ALWAYS;
                };

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

        final Trigger atReef = swerve.atPoseAndStoppedTrigger(scoringPoseSupplier);
        final Trigger atCloseReef = swerve.atPoseTrigger(
                scoringPoseSupplier,
                new HolonomicDriveController.PositionTolerance(
                        0.5,
                        Rotation2d.fromDegrees(8)
                ),
                new HolonomicDriveController.VelocityTolerance(
                        0.25,
                        Math.PI / 2
                )
        );
        final Trigger atRotationCloseReef = atRotationTrigger(
                scoringPoseSupplier,
                Rotation2d.fromDegrees(25),
                Math.PI / 3
        );
        final Trigger always = new Trigger(() -> true);

        final Container<Boolean> wasEverAtReef = Container.of(false);

        final Map<ExtendWhen, Command> extendWhenCommandMap = Map.of(
                ExtendWhen.CLOSE, Commands.waitUntil(atCloseReef),
                ExtendWhen.ROTATION_CLOSE, Commands.waitUntil(atRotationCloseReef),
                ExtendWhen.ALWAYS, Commands.none()
        );
        final Supplier<Trigger> extendWhenTriggerSupplier = () ->
                switch (extendWhenSupplier.get()) {
                    case CLOSE -> atCloseReef;
                    case ROTATION_CLOSE -> atRotationCloseReef;
                    case ALWAYS -> always;
                };
        final BooleanSupplier willWaitToExtend = () ->
                !extendWhenTriggerSupplier.get().getAsBoolean();

        final Trigger atSuperstructureSetpoint = superstructure
                .atSetpoint(() -> scorePositionContainer.value.level.goal);

        final Container<Superstructure.Goal> superstructureGoalContainer = Container.empty();
        final Runnable setSuperstructureGoalToAlign = () -> superstructureGoalContainer.value
                = Superstructure.Goal.getAlignGoal(scorePositionContainer.value.level.goal);
        final Runnable setSuperstructureGoalToScore = () -> superstructureGoalContainer.value
                = scorePositionContainer.value.level.goal;

        return Commands.sequence(
                Commands.runOnce(updateScorePosition),
                Commands.runOnce(updateScoringPoseMap),
                wasEverAtReef.set(false),
                Commands.either(
                        Commands.runOnce(setSuperstructureGoalToAlign),
                        Commands.runOnce(setSuperstructureGoalToScore),
                        shouldUseEarlyAlign
                ),
                superstructure.toInstantGoal(Superstructure.Goal.STOW)
                        .onlyIf(superstructure.unsafeToDrive),
                Commands.deadline(
                        Commands.sequence(
                                Commands.deadline(
                                        Commands.repeatingSequence(
                                                runningScorePositionContainer.set(scorePositionContainer),
                                                Commands.sequence(
                                                        Commands.sequence(
                                                                Commands.either(
                                                                        Commands.runOnce(setSuperstructureGoalToAlign),
                                                                        Commands.runOnce(setSuperstructureGoalToScore),
                                                                        shouldUseEarlyAlign
                                                                ),
                                                                Commands.select(
                                                                        extendWhenCommandMap,
                                                                        extendWhenSupplier
                                                                )
                                                        ).onlyIf(willWaitToExtend),
                                                        Commands.run(setSuperstructureGoalToScore)
                                                ).onlyWhile(
                                                        () -> runningScorePositionContainer.value
                                                                == scorePositionContainer.value
                                                )
                                        ).until(atReef),
                                        Commands.run(updateScorePosition)
                                ),
                                Commands.runOnce(setSuperstructureGoalToScore),
                                Commands.waitUntil(atReef.or(wasEverAtReef::get)),
                                Commands.waitUntil(atSuperstructureSetpoint)
                                        .withTimeout(2),
                                intake.scoreCoral(scoreModeFromScorePosition(scorePositionSupplier)).asProxy()
                        ),
                        Commands.sequence(
                                swerve.runToPose(scoringPoseSupplier)
                                        .until(atReef),
                                wasEverAtReef.set(true),
                                swerve.runWheelXCommand()
                        ),
                        superstructure.toGoal(superstructureGoalContainer)
                ),
                Commands.waitUntil(superstructure.unsafeToDrive.negate())
                        .withTimeout(0.5)
        ).withName("ScoreAtFixedPosition");
    }

    public Command readyScoreAtPositionNoLineup(final Supplier<ScorePosition> scorePositionSupplier) {
        return superstructure.runGoal(() -> scorePositionSupplier.get().level().goal)
                .withName("ReadyScoreAtPositionNoLineup");
    }

    public Command descoreUpperAlgae() {
        final Supplier<Pose2d> descorePoseSupplier = () -> swerve.getPose().
                nearest(new ArrayList<>(FieldConstants.getReefCenterPoses().values()))
                .plus(FieldConstants.ALGAE_DESCORE_DISTANCE_OFFSET);

        return Commands.deadline(
                Commands.sequence(
                        swerve.runToPose(descorePoseSupplier)
                                .until(gamepieceState.hasAlgae),
                        Commands.waitSeconds(0.2),
                        swerve.drive(() -> -0.8, () -> 0, () -> 0, false, false)
                                .withTimeout(0.35)
                ),
                superstructure.toGoal(Superstructure.Goal.UPPER_ALGAE),
                intake.intakeAlgae().asProxy()
        ).withName("DescoreUpperAlgae");
    }

    public Command descoreLowerAlgae() {
        final Supplier<Pose2d> descorePoseSupplier = () -> swerve.getPose().
                nearest(new ArrayList<>(FieldConstants.getReefCenterPoses().values()))
                .plus(FieldConstants.ALGAE_DESCORE_DISTANCE_OFFSET);

        return Commands.deadline(
                Commands.sequence(
                        swerve.runToPose(descorePoseSupplier)
                                .until(gamepieceState.hasAlgae),
                        Commands.waitSeconds(0.2),
                        swerve.drive(() -> -0.8, () -> 0, () -> 0, false, false)
                                .withTimeout(0.35)
                ),
                superstructure.toGoal(Superstructure.Goal.LOWER_ALGAE),
                intake.intakeAlgae().asProxy()
        ).withName("DescoreLowerAlgae");
    }

    public Command scoreAtPosition(final Supplier<ScorePosition> scorePositionSupplier) {
        final Container<Superstructure.Goal> goalContainer = Container.empty();

        return Commands.deadline(
                Commands.sequence(
                        goalContainer.set(() -> scorePositionSupplier.get().level.goal),
                        Commands.deadline(
                                Commands.waitUntil(superstructure.atSetpoint(goalContainer))
                                        .withTimeout(3)
                                        .andThen(intake.scoreCoral(scoreModeFromScorePosition(scorePositionSupplier))),
                                superstructure.toGoal(goalContainer)
                        ),
                        Commands.waitUntil(superstructure.unsafeToDrive.negate()
                                .or(superstructure.atSetpoint(Superstructure.Goal.STOW)))
                ),
                swerve.runWheelXCommand()
        ).withName("ScoreAtPositionTeleop");
    }

    public Command scoreBarge() {
        final Container<Superstructure.Goal> superstructureGoal = Container.empty();

        final DoubleSupplier axisTarget = () -> FieldConstants.getScoringBargeCenterCage().getX();
        final DoubleSupplier robotX = () -> swerve.getPose().getX();
        final DoubleSupplier robotY = () -> swerve.getPose().getY();

        final Trigger allowedToScore = new Trigger(() -> {
            if (Robot.IsRedAlliance.getAsBoolean()) {
                return robotY.getAsDouble() < FieldConstants.FIELD_WIDTH_Y_METERS / 2.0;
            } else {
                return robotY.getAsDouble() > FieldConstants.FIELD_WIDTH_Y_METERS / 2.0;
            }
        });

        return Commands.sequence(
                superstructureGoal.set(Superstructure.Goal.STOW),
                Commands.deadline(
                        Commands.sequence(
                                Commands.parallel(
                                        superstructureGoal.set(Superstructure.Goal.ALIGN_NET),
                                        swerve.driveToAxisFacingAngle(
                                                axisTarget,
                                                Swerve.DriveAxis.X,
                                                () -> Robot.IsRedAlliance.getAsBoolean()
                                                        ? Rotation2d.kPi
                                                        : Rotation2d.kZero
                                        )
                                ).onlyIf(swerve.atAxisTrigger(axisTarget, robotX).negate()),
                                swerve.wheelXCommand(),
                                superstructureGoal.set(Superstructure.Goal.NET),
                                Commands.waitUntil(superstructure.extendedBeyond(0.47)),
                                intake.netAlgae()
                        ),
                        superstructure.toGoal(superstructureGoal)
                )
        )
                .onlyIf(allowedToScore)
                .withName("ScoreNetFlingFacingBarge");
    }

    public Command scoreManualBarge() {
        final Container<Superstructure.Goal> superstructureGoal = Container.empty();

        return Commands.sequence(
                superstructureGoal.set(Superstructure.Goal.STOW),
                Commands.deadline(
                        Commands.sequence(
                                superstructureGoal.set(Superstructure.Goal.ALIGN_NET),
                                superstructureGoal.set(Superstructure.Goal.NET),
                                Commands.waitUntil(superstructure.extendedBeyond(0.47)),
                                intake.netAlgae()
                        ),
                        superstructure.toGoal(superstructureGoal)

                )
        );
    }

    public Command readyScoreProcessor() {
        return superstructure.runGoal(Superstructure.Goal.PROCESSOR)
                .withName("ReadyScoreProcessor");
    }

    public Command scoreProcessor() {
        return Commands.deadline(
                Commands.sequence(
                        Commands.waitUntil(superstructure.atSetpoint(Superstructure.Goal.PROCESSOR))
                                .withTimeout(2),
                        intake.scoreAlgae(),
                        Commands.waitUntil(intake.isCurrentAboveAlgaeThreshold.negate()).withTimeout(2)
                ),
                superstructure.toGoal(Superstructure.Goal.PROCESSOR),
                swerve.runWheelXCommand()
        ).withName("ScoreProcessor");
    }

    public Command intakeLowerAlgae() {
        return Commands.parallel(
                superstructure.toGoal(Superstructure.Goal.LOWER_ALGAE),
                intake.intakeAlgae().asProxy()
        ).withName("IntakeLowerAlgae");
    }

    public Command intakeUpperAlgae() {
        return Commands.parallel(
                superstructure.toGoal(Superstructure.Goal.UPPER_ALGAE),
                intake.intakeAlgae().asProxy()
        ).withName("IntakeUpperAlgae");
    }

    public Command intakeAlgaeFromGround() {
        return Commands.parallel(
                superstructure.toGoal(Superstructure.Goal.ALGAE_GROUND),
                intake.intakeAlgae()
        ).withName("IntakeAlgaeFromGround");
    }

    public Command processor() {
        final Supplier<Pose2d> alignPoseSupplier = FieldConstants::getProcessorAlignPose;

        final Trigger atAlignProcessor = swerve.atPoseTrigger(
                alignPoseSupplier,
                new HolonomicDriveController.PositionTolerance(
                        0.2,
                        Rotation2d.fromDegrees(10)
                ),
                new HolonomicDriveController.VelocityTolerance(
                        0.2,
                        Units.degreesToRadians(20)
                )
        );

        return Commands.deadline(
                Commands.sequence(
                        swerve.runToPose(alignPoseSupplier)
                                .until(atAlignProcessor),
                        swerve.driveToPose(FieldConstants::getProcessorScoringPose),
                        Commands.parallel(
                                swerve.runWheelXCommand(),
                                intake.scoreAlgae()
                        )
                ),
                superstructure.toGoal(Superstructure.Goal.PROCESSOR)
        );
    }

    @SuppressWarnings("SuspiciousNameCombination")
    public Command readyClimb(
            final DoubleSupplier leftStickYInput,
            final DoubleSupplier leftStickXInput
    ) {
        return Commands.parallel(
                superstructure.runGoal(Superstructure.Goal.CLIMB),
                swerve.teleopFacingAngle(
                        leftStickYInput,
                        leftStickXInput,
                        () -> Robot.IsRedAlliance.getAsBoolean() ? Rotation2d.kZero : Rotation2d.k180deg
                )
        ).withName("ReadyClimb");
    }

    public Command climb() {
        final DoubleSupplier climbAxisX = () -> FieldConstants.getCenterCage().getX();

        return Commands.sequence(
                swerve.driveToAxisFacingAngle(
                        climbAxisX,
                        Swerve.DriveAxis.X,
                        () -> Robot.IsRedAlliance.getAsBoolean() ? Rotation2d.kZero : Rotation2d.k180deg
                ),
                superstructure.toInstantGoal(Superstructure.Goal.CLIMB_DOWN)
        ).withName("Climb");
    }
}