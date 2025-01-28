package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.FieldConstants.Reef;
import frc.robot.state.GamepieceState;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.superstructure.Profiles;
import frc.robot.subsystems.superstructure.SplineProfile;
import frc.robot.subsystems.superstructure.Superstructure;

import java.util.*;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class ScoreCommands {
    public enum Level {
        L1(Reef.Level.L1, Superstructure.Goal.L1),
        L2(Reef.Level.L2, Superstructure.Goal.L2),
        L3(Reef.Level.L3, Superstructure.Goal.L3),
        L4(Reef.Level.L4, Superstructure.Goal.L4);

        public static final Map<Reef.Level, Level> LevelMap = new HashMap<>();

        static {
            for (final Level level : Level.values()) {
                LevelMap.put(level.level, level);
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

    private static final List<Superstructure.Goal> ScoreGoals = List.of(
            Superstructure.Goal.DYNAMIC,
            Superstructure.Goal.L1,
            Superstructure.Goal.L2,
            Superstructure.Goal.L3,
            Superstructure.Goal.L4
    );

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

    public Supplier<ScorePosition> getScorePositionSupplier(
            final DoubleSupplier rightStickXInput,
            final DoubleSupplier rightStickYInput
    ) {
        return () -> {
            final Reef.Side side;
            final Level level;

            if (rightStickXInput.getAsDouble() > 0) {
                side = Reef.Side.RIGHT;
            } else {
                side = Reef.Side.LEFT;
            }

            final double yStickPosition = -rightStickYInput.getAsDouble();
            if (yStickPosition >= 0.75) {
                level = Level.L4;
            } else if (yStickPosition >= 0.25) {
                level = Level.L3;
            } else if (yStickPosition >= -0.5) {
                level = Level.L2;
            } else {
                level = Level.L1;
            }

            return new ScorePosition(side, level);
        };
    }

    public Command intakeFacingClosestCoralStation(
            final DoubleSupplier leftStickXInput,
            final DoubleSupplier leftStickYInput
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
                intake.runCoralRollerVelocity(3)
        );
    }

    private Superstructure.Goal getNonDynamicGoal() {
        final Superstructure.Goal maybeDynamicSuperstructureGoal =
                superstructure.getCurrentSuperstructureGoal();

        return maybeDynamicSuperstructureGoal == Superstructure.Goal.DYNAMIC
                ? superstructure.getClosestGoal().orElseThrow()
                : maybeDynamicSuperstructureGoal;
    }

    public Command readyScoreAtPosition(
            final Supplier<ScorePosition> scorePositionSupplier,
            final DoubleSupplier coralDistanceOffset
    ) {
        return Commands.parallel(
                Commands.defer(
                        () -> {
                            final Pose2d currentPose = swerve.getPose();
                            final Translation2d currentTranslation = currentPose.getTranslation();
                            final Pose2d[] reefCenterPoses = FieldConstants.getReefScoringCenterPoses();
                            final List<Map<Reef.Side, Map<Reef.Level, Pose2d>>> branchScoringPositions =
                                    FieldConstants.getBranchScoringPositions();

                            Map<Reef.Side, Map<Reef.Level, Pose2d>> scoringPoseMap = null;
                            double closestDistanceMeters = Double.MAX_VALUE;
                            for (int i = 0; i < reefCenterPoses.length; i++) {
                                final Pose2d centerFace = reefCenterPoses[i];

                                final double distanceMeters = centerFace.getTranslation()
                                        .getDistance(currentTranslation);

                                if (distanceMeters < closestDistanceMeters) {
                                    scoringPoseMap = branchScoringPositions.get(i);
                                    closestDistanceMeters = distanceMeters;
                                }
                            }

                            if (scoringPoseMap == null) {
                                return Commands.none();
                            }

                            final Map<Reef.Side, Map<Reef.Level, Pose2d>> finalScoringPoseMap = scoringPoseMap;
                            return swerve.runToPose(() -> {
                                final ScorePosition scorePosition = scorePositionSupplier.get();
                                final Pose2d scoringPose = finalScoringPoseMap
                                        .get(scorePosition.side)
                                        .get(scorePosition.level.level);

                                return scoringPose.transformBy(
                                        new Transform2d(
                                                coralDistanceOffset.getAsDouble(),
                                                0,
                                                Rotation2d.kZero
                                        )
                                );
                            });
                        },
                        Set.of(swerve)
                ),
                Commands.sequence(
                        Commands.waitUntil(swerve.atHolonomicDrivePose),
                        Commands.repeatingSequence(
                                Commands.either(
                                        Commands.defer(
                                                () -> {
                                                    final ScorePosition scorePosition = scorePositionSupplier.get();
                                                    final Superstructure.Goal superstructureGoal = getNonDynamicGoal();

                                                    final Optional<SplineProfile> splineProfile = Profiles.getProfile(
                                                            superstructureGoal,
                                                            scorePosition.level.goal
                                                    );

                                                    final BooleanSupplier desiredScorePositionNotEqualLast =
                                                            () -> !scorePosition.equals(scorePositionSupplier.get());

                                                    if (splineProfile.isEmpty()) {
                                                        return Commands.idle().until(desiredScorePositionNotEqualLast);
                                                    } else {
                                                        return superstructure.runProfile(splineProfile.get())
                                                                .until(desiredScorePositionNotEqualLast);
                                                    }
                                                },
                                                superstructure.getRequirements()
                                        ),
                                        Commands.defer(
                                                () -> superstructure.runWaitSuperstructureGoal(scorePositionSupplier.get().level.goal),
                                                superstructure.getRequirements()
                                        ),
                                        () -> ScoreCommands.ScoreGoals.contains(superstructure.getCurrentSuperstructureGoal())
                                )
                        )
                )
        );
    }

    public Command scoreAtPosition(final Supplier<ScorePosition> scorePosition) {
        return Commands.deadline(
                Commands.sequence(
                        Commands.waitUntil(superstructure.atSuperstructureSetpoint).withTimeout(2),
                        intake.scoreCoral()
                ),
                Commands.defer(
                        () -> superstructure.toSuperstructureGoal(scorePosition.get().level.goal),
                        superstructure.getRequirements()
                ),
                swerve.runWheelXCommand()
        );
    }

    public Command readyScoreProcessor() {
        return Commands.parallel(
                swerve.driveToPose(FieldConstants::getProcessorScoringPose),
                superstructure.runSuperstructureGoal(Superstructure.Goal.PROCESSOR)
        );
    }

    public Command scoreProcessor() {
        return Commands.deadline(
                Commands.sequence(
                        Commands.waitUntil(superstructure.atSuperstructureSetpoint),
                        intake.runAlgaeRollerVoltage(-8),
                        Commands.waitUntil(intake.isAlgaePresent.negate())
                ),
                superstructure.toSuperstructureGoal(Superstructure.Goal.PROCESSOR),
                swerve.runWheelXCommand()
        );
    }

    public Command readyIntakeAlgaeAtPosition() {
        return Commands.defer(
                () -> {
                    final Pose2d currentPose = swerve.getPose();
                    final Pose2d[] reefCenterPoses = FieldConstants.getReefScoringCenterPoses();
                    final Pose2d nearestCoralSide = currentPose.nearest(List.of(reefCenterPoses));

                    return swerve.runToPose(() -> nearestCoralSide);
                },
                Set.of(swerve)
        );
    }
    public Command intakeLowerAlgae() {
        return Commands.deadline(
                Commands.sequence(
                        Commands.waitUntil(superstructure.atSuperstructureSetpoint),
                        intake.runAlgaeRollerVoltage(8),
                        Commands.waitUntil(intake.isAlgaePresent)
                ),
                superstructure.toSuperstructureGoal(Superstructure.Goal.LOWER_ALGAE),
                swerve.runWheelXCommand()
        );
    }

    public Command intakeUpperAlgae() {
        return Commands.deadline(
                Commands.sequence(
                        Commands.waitUntil(superstructure.atSuperstructureSetpoint),
                        intake.runAlgaeRollerVoltage(8),
                        Commands.waitUntil(intake.isAlgaePresent)
                ),
                superstructure.toSuperstructureGoal(Superstructure.Goal.UPPER_ALGAE),
                swerve.runWheelXCommand()
        );
    }

    public Command readyScoreNet(
            final DoubleSupplier leftStickXInput,
            final DoubleSupplier leftStickYInput
    ) {
        return Commands.parallel(
                swerve.teleopFacingAngleCommand(
                        leftStickYInput,
                        leftStickXInput,
                        () -> Robot.IsRedAlliance.getAsBoolean() ? Rotation2d.kZero : Rotation2d.kPi
                ),
                superstructure.runSuperstructureGoal(Superstructure.Goal.NET)
        );
    }

    public Command scoreNet() {
        return Commands.deadline(
                Commands.sequence(
                        Commands.waitUntil(superstructure.atSuperstructureSetpoint),
                        intake.runAlgaeRollerVoltage(-8),
                        Commands.waitUntil(intake.isAlgaePresent.negate())
                ),
                superstructure.toSuperstructureGoal(Superstructure.Goal.NET),
                swerve.runWheelXCommand()
        );
    }
}