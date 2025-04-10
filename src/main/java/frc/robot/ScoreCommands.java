package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
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
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

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
    ) {}

    private static final double AllowableDistanceAlgaeFromReefMeters = 0.22;

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
        return Commands.parallel(
                swerve.teleopFacingAngle(
                        leftStickYInput,
                        leftStickXInput,
                        () -> {
                            final Pose2d currentPose = swerve.getPose();
                            final Pose2d nearestStation = currentPose.nearest(FieldConstants.getHPPickupPoses());

                            return nearestStation.getRotation();
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
                    case L4 -> true;
                    case L3, L2, L1 -> false;
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

        final Trigger atReef = swerve.atPoseAndStoppedTrigger(scoringPoseSupplier);
        final Trigger atCloseEnoughReef = swerve.atPoseTrigger(
                scoringPoseSupplier,
                new HolonomicDriveController.PositionTolerance(
                        0.35,
                        Rotation2d.fromDegrees(8)
                ),
                new HolonomicDriveController.VelocityTolerance(
                        0.25,
                        Math.PI / 2
                )
        );
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
                Commands.either(
                        Commands.runOnce(setSuperstructureGoalToAlign),
                        Commands.runOnce(setSuperstructureGoalToScore),
                        shouldUseEarlyAlign
                ),
                superstructure.toInstantGoal(Superstructure.Goal.STOW)
                        .onlyIf(superstructure.unsafeToDrive),
                Commands.deadline(
                        Commands.sequence(
                                Commands.waitUntil(atCloseEnoughReef),
                                Commands.runOnce(setSuperstructureGoalToScore),
                                Commands.waitUntil(atReef),
                                Commands.waitUntil(atSuperstructureSetpoint)
                                        .withTimeout(2),
                                intake.scoreCoral()
                        ),
                        Commands.sequence(
                                swerve.runToPose(scoringPoseSupplier)
                                        .until(atReef),
                                swerve.runWheelXCommand()
                        ),
                        Commands.parallel(
                                Commands.run(updateScorePosition),
                                Commands.either(
                                        Commands.runOnce(setSuperstructureGoalToAlign)
                                                .andThen(Commands.idle())
                                                .onlyWhile(shouldUseEarlyAlign),
                                        Commands.runOnce(setSuperstructureGoalToScore)
                                                .andThen(Commands.idle())
                                                .onlyWhile(shouldUseEarlyAlign.negate()),
                                        shouldUseEarlyAlign
                                ).repeatedly()
                        ).until(atCloseEnoughReef),
                        superstructure.toGoal(superstructureGoalContainer)
                ),
                Commands.waitUntil(superstructure.unsafeToDrive.negate())
                        .withTimeout(0.8)
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

        final Trigger farEnoughAwayFromReef = new Trigger(() -> swerve.getPose().getTranslation()
                .getDistance(descorePoseSupplier.get().getTranslation()) >= AllowableDistanceAlgaeFromReefMeters);

        return Commands.deadline(
                Commands.sequence(
                        swerve.runToPose(descorePoseSupplier)
                                .until(gamepieceState.hasAlgae),
                        Commands.waitSeconds(0.2),
                        swerve.drive(() -> -0.7, () -> 0, () -> 0, false, false)
                                .until(farEnoughAwayFromReef)
                ),
                superstructure.toGoal(Superstructure.Goal.UPPER_ALGAE),
                intake.intakeAlgae().asProxy()
        ).withName("DescoreUpperAlgae");
    }

    public Command descoreLowerAlgae() {
        final Supplier<Pose2d> descorePoseSupplier = () -> swerve.getPose().
                nearest(new ArrayList<>(FieldConstants.getReefCenterPoses().values()))
                .plus(FieldConstants.ALGAE_DESCORE_DISTANCE_OFFSET);

        final Trigger farEnoughAwayFromReef = new Trigger(() -> swerve.getPose().getTranslation()
                .getDistance(descorePoseSupplier.get().getTranslation()) >= AllowableDistanceAlgaeFromReefMeters);

        return Commands.deadline(
                Commands.sequence(
                        swerve.runToPose(descorePoseSupplier)
                                .until(gamepieceState.hasAlgae),
                        Commands.waitSeconds(0.2),
                        swerve.drive(() -> -0.7, () -> 0, () -> 0, false, false)
                                .until(farEnoughAwayFromReef)
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
                                        .andThen(intake.scoreCoral()),
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
        ).withName("ScoreNetFlingFacingBarge");
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
        return Commands.deadline(
                Commands.sequence(
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