package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.FieldConstants.Reef;
import frc.robot.state.GamepieceState;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.utils.Container;

import java.util.HashMap;
import java.util.Map;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class ScoreCommands {
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

    public Pose2d offsetScoringPoseWithCANRange(final Pose2d targetPose) {
        final Transform2d coralDistanceOffset = new Transform2d(
                0,
                gamepieceState.hasCoral.getAsBoolean()
                        ? intake.coralDistanceIntakeCenterMeters.getAsDouble() - Units.inchesToMeters(1)
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
                swerve.teleopFacingAngleCommand(
                        leftStickYInput,
                        leftStickXInput,
                        () -> {
                            final Pose2d currentPose = swerve.getPose();
                            final Pose2d nearestStation;
                            if (Robot.IsRedAlliance.getAsBoolean()) {
                                nearestStation = currentPose.nearest(
                                        FieldConstants.CoralStation.RED_CORAL_STATIONS
                                );
                            } else {
                                nearestStation = currentPose.nearest(
                                        FieldConstants.CoralStation.BLUE_CORAL_STATIONS
                                );
                            }
                            return nearestStation.getRotation().rotateBy(Rotation2d.kPi);
                        }
                ),
                superstructure.toSuperstructureGoal(Superstructure.Goal.HP),
                intake.intakeCoralHP()
        );
    }

    public Command readyScoreAtPosition(final Supplier<ScorePosition> wantScorePosition) {
        final Container<ScorePosition> driveToScorePosition = Container.of(wantScorePosition.get());
        final Supplier<ScorePosition> driveToScorePositionSupplier = () -> driveToScorePosition.value;
        final Consumer<ScorePosition> setDriveToScorePosition =
                (scorePosition) -> driveToScorePosition.value = scorePosition;

        return Commands.parallel(
                Commands.defer(
                        () -> {
                            final Pose2d currentPose = swerve.getPose();
                            final Translation2d currentTranslation = currentPose.getTranslation();
                            final Map<Reef.Face, Pose2d> reefCenterPoses = FieldConstants.getReefScoringCenterPoses();
                            final Map<Reef.Face, Map<Reef.Side, Map<Reef.Level, Pose2d>>> branchScoringPositions =
                                    FieldConstants.getBranchScoringPositions();

                            Map<Reef.Side, Map<Reef.Level, Pose2d>> scoringPoseMap = null;
                            double closestDistanceMeters = Double.MAX_VALUE;
                            for (final Reef.Face face : Reef.Face.values()) {
                                final Pose2d centerFace = reefCenterPoses.get(face);

                                final double distanceMeters = centerFace.getTranslation()
                                        .getDistance(currentTranslation);

                                if (distanceMeters < closestDistanceMeters) {
                                    scoringPoseMap = branchScoringPositions.get(face);
                                    closestDistanceMeters = distanceMeters;
                                }
                            }

                            if (scoringPoseMap == null) {
                                return Commands.none();
                            }

                            //noinspection ExtractMethodRecommender
                            final Map<Reef.Side, Map<Reef.Level, Pose2d>> finalScoringPoseMap = scoringPoseMap;
                            final Supplier<Pose2d> scoringPoseSupplier = () -> {
                                final ScorePosition scorePosition = driveToScorePositionSupplier.get();

                                final Pose2d scoringPose = finalScoringPoseMap
                                        .get(scorePosition.side)
                                        .get(scorePosition.level.level);
                                final Transform2d coralDistanceOffset = new Transform2d(
                                        0,
                                        gamepieceState.hasCoral.getAsBoolean()
                                                ? intake.coralDistanceIntakeCenterMeters.getAsDouble()
                                                : 0,
                                        Rotation2d.kZero
                                );

                                return scoringPose.transformBy(coralDistanceOffset);
                            };

                            return Commands.sequence(
                                    swerve.driveToPose(() -> scoringPoseSupplier
                                            .get()
                                            .transformBy(FieldConstants.ALIGN_DISTANCE_OFFSET_METERS)
                                    ),
                                    swerve.runToPose(scoringPoseSupplier)
                            );
                        },
                        Set.of(swerve)
                ),
                Commands.sequence(
                        Commands.runOnce(() -> setDriveToScorePosition.accept(wantScorePosition.get())),
                        Commands.waitUntil(swerve.atHolonomicDrivePose),
                        Commands.repeatingSequence(
                                Commands.defer(() -> {
                                    final ScorePosition scorePosition = wantScorePosition.get();
                                    final BooleanSupplier desiredScorePositionNotEqualLast =
                                            () -> !scorePosition.equals(wantScorePosition.get());

                                    return Commands.sequence(
                                            superstructure.runSuperstructureGoal(Superstructure.Goal.SAFE)
                                                    .until(superstructure.atSuperstructureSetpoint
                                                            .and(superstructure.unsafeToDrive.negate()))
                                                    .onlyIf(superstructure.unsafeToDrive),
                                            Commands.runOnce(() -> setDriveToScorePosition.accept(scorePosition)),
                                            Commands.waitUntil(swerve.atHolonomicDrivePose),
                                            superstructure.runSuperstructureGoal(scorePosition.level.goal)
                                    ).until(desiredScorePositionNotEqualLast);
                                }, superstructure.getRequirements())
                        )
                )
        ).withName("ReadyScoreAtPosition");
    }

    public Command readyScoreAtPositionNoLineup(final Supplier<ScorePosition> scorePositionSupplier) {
        return superstructure.runSuperstructureGoal(() -> scorePositionSupplier.get().level().goal);
    }

    public Command scoreAtPosition() {
        return Commands.sequence(
                Commands.deadline(
                        Commands.sequence(
                                Commands.waitUntil(superstructure.atSuperstructureSetpoint)
                                        .withTimeout(3),
                                intake.scoreCoral()
                        ),
                        Commands.defer(
                                () -> superstructure.toSuperstructureGoal(superstructure.getDesiredSuperstructureGoal()),
                                superstructure.getRequirements()
                        ),
                        swerve.runWheelXCommand()
                ),
                Commands.waitSeconds(1)
        ).withName("ScoreAtPositionTeleop");
    }

    public Command readyScoreProcessor() {
        return superstructure.runSuperstructureGoal(Superstructure.Goal.PROCESSOR);
    }

    public Command scoreProcessor() {
        return Commands.deadline(
                Commands.sequence(
                        Commands.waitUntil(superstructure.atSuperstructureSetpoint).withTimeout(2),
                        intake.scoreAlgae(),
                        Commands.waitUntil(intake.isAlgaePresent.negate()).withTimeout(2)
                ),
                superstructure.toSuperstructureGoal(Superstructure.Goal.PROCESSOR),
                swerve.runWheelXCommand()
        );
    }

    public Command intakeLowerAlgae() {
        return Commands.parallel(
                superstructure.toSuperstructureGoal(Superstructure.Goal.LOWER_ALGAE),
                intake.intakeAlgae().asProxy()
        );
    }

    public Command intakeUpperAlgae() {
        return Commands.parallel(
                superstructure.toSuperstructureGoal(Superstructure.Goal.UPPER_ALGAE),
                intake.intakeAlgae().asProxy()
        );
    }

    public Command intakeAlgaeFromGround() {
        return Commands.parallel(
                superstructure.toSuperstructureGoal(Superstructure.Goal.ALGAE_GROUND),
                intake.intakeAlgae()
        );
    }
}