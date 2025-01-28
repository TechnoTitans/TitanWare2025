package frc.robot.constants;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class FieldConstants {
    //borrowed from mechanical advantage :)
    public static final double FIELD_LENGTH_X_METERS = Units.inchesToMeters(690.876);
    public static final double FIELD_WIDTH_Y_METERS = Units.inchesToMeters(317);
    public static final Pose2d RED_ORIGIN = new Pose2d(FIELD_LENGTH_X_METERS, FIELD_WIDTH_Y_METERS, Rotation2d.k180deg);

    public static class Processor {
        public static final Pose2d BLUE_CENTER_FACE =
                new Pose2d(Units.inchesToMeters(235.726), 0, Rotation2d.fromDegrees(90));
        public static final Pose2d RED_CENTER_FACE = BLUE_CENTER_FACE.relativeTo(RED_ORIGIN);

        public static final Pose2d BLUE_SCORING_POSE = BLUE_CENTER_FACE.transformBy(
                new Transform2d(Units.inchesToMeters(20), 0, Rotation2d.k180deg)
        );
        public static final Pose2d RED_SCORING_POSE = RED_CENTER_FACE.transformBy(
                new Transform2d(Units.inchesToMeters(20), 0, Rotation2d.k180deg)
        );
    }

    public static class Barge {
        public static final Pose2d BLUE_LEFT_CAGE =
                new Pose2d(FIELD_LENGTH_X_METERS / 2, Units.inchesToMeters(286.779), Rotation2d.kZero);
        public static final Pose2d BLUE_MIDDLE_CAGE =
                new Pose2d(FIELD_LENGTH_X_METERS / 2, Units.inchesToMeters(242.855), Rotation2d.kZero);
        public static final Pose2d BLUE_RIGHT_CAGE =
                new Pose2d(FIELD_LENGTH_X_METERS / 2, Units.inchesToMeters(199.947), Rotation2d.kZero);

        public static final Pose2d RED_LEFT_CAGE = BLUE_LEFT_CAGE.relativeTo(RED_ORIGIN);
        public static final Pose2d RED_MIDDLE_CAGE = BLUE_MIDDLE_CAGE.relativeTo(RED_ORIGIN);
        public static final Pose2d RED_RIGHT_CAGE = BLUE_RIGHT_CAGE.relativeTo(RED_ORIGIN);
    }

    public static class CoralStation {
        public static final Pose2d BLUE_LEFT_CENTER_FACE =
                new Pose2d(
                        Units.inchesToMeters(33.526),
                        Units.inchesToMeters(291.176),
                        Rotation2d.fromDegrees(90 - 144.011));
        public static final Pose2d BLUE_RIGHT_CENTER_FACE =
                new Pose2d(
                        Units.inchesToMeters(33.526),
                        Units.inchesToMeters(25.824),
                        Rotation2d.fromDegrees(144.011 - 90));
        public static final List<Pose2d> BLUE_CORAL_STATIONS = List.of(
                BLUE_LEFT_CENTER_FACE,
                BLUE_RIGHT_CENTER_FACE
        );

        public static final Pose2d RED_LEFT_CENTER_FACE = BLUE_LEFT_CENTER_FACE.relativeTo(RED_ORIGIN);
        public static final Pose2d RED_RIGHT_CENTER_FACE = BLUE_RIGHT_CENTER_FACE.relativeTo(RED_ORIGIN);
        public static final List<Pose2d> RED_CORAL_STATIONS = List.of(
                RED_LEFT_CENTER_FACE,
                RED_RIGHT_CENTER_FACE
        );
    }

    public static class Reef {
        public enum Side {
            LEFT,
            RIGHT
        }

        public enum Level {
            L1(Units.inchesToMeters(18), 0),
            L2(Units.inchesToMeters(31.875), -35),
            L3(Units.inchesToMeters(47.625), -35),
            L4(Units.inchesToMeters(72), -90);

            public final double heightMeters;
            public final double pitchDegrees;
            Level(final double heightMeters, double pitchDegrees) {
                this.heightMeters = heightMeters;
                this.pitchDegrees = pitchDegrees;
            }
        }

        public static final Translation2d BLUE_CENTER =
                new Translation2d(Units.inchesToMeters(176.746), Units.inchesToMeters(158.501));
        public static final Translation2d RED_CENTER = BLUE_CENTER.plus(
                new Translation2d(-FIELD_LENGTH_X_METERS, 0)
        );

        // Side of the reef to the inside of the reef zone line
        public static final double faceToZoneLine = Units.inchesToMeters(12);

        // Starting facing the driver station in clockwise order
        public static final Pose2d[] BLUE_CENTER_FACES = new Pose2d[6];
        public static final Pose2d[] BLUE_CENTER_SCORING_FACES = new Pose2d[6];
        public static final Pose2d[] RED_CENTER_FACES = new Pose2d[6];
        public static final Pose2d[] RED_CENTER_SCORING_FACES = new Pose2d[6];
        // Starting at the right branch facing the driver station in clockwise
        public static final List<Map<Side, Map<Level, Pose3d>>> BLUE_BRANCH_POSITIONS = new ArrayList<>();
        public static final List<Map<Side, Map<Level, Pose3d>>> RED_BRANCH_POSITIONS = new ArrayList<>();
        public static final List<Map<Side, Map<Level, Pose2d>>> BLUE_BRANCH_SCORING_POSITIONS = new ArrayList<>();
        public static final List<Map<Side, Map<Level, Pose2d>>> RED_BRANCH_SCORING_POSITIONS = new ArrayList<>();

        static {
            BLUE_CENTER_FACES[0] =
                    new Pose2d(
                            Units.inchesToMeters(144.003),
                            Units.inchesToMeters(158.500),
                            Rotation2d.fromDegrees(180));
            BLUE_CENTER_FACES[1] =
                    new Pose2d(
                            Units.inchesToMeters(160.373),
                            Units.inchesToMeters(186.857),
                            Rotation2d.fromDegrees(120));
            BLUE_CENTER_FACES[2] =
                    new Pose2d(
                            Units.inchesToMeters(193.116),
                            Units.inchesToMeters(186.858),
                            Rotation2d.fromDegrees(60));
            BLUE_CENTER_FACES[3] =
                    new Pose2d(
                            Units.inchesToMeters(209.489),
                            Units.inchesToMeters(158.502),
                            Rotation2d.fromDegrees(0));
            BLUE_CENTER_FACES[4] =
                    new Pose2d(
                            Units.inchesToMeters(193.118),
                            Units.inchesToMeters(130.145),
                            Rotation2d.fromDegrees(-60));
            BLUE_CENTER_FACES[5] =
                    new Pose2d(
                            Units.inchesToMeters(160.375),
                            Units.inchesToMeters(130.144),
                            Rotation2d.fromDegrees(-120));



            final Pose3d RED_ORIGIN_POSE3D = new Pose3d(RED_ORIGIN);
            final Transform2d reefFaceScoringTransform = new Transform2d(Units.inchesToMeters(20), 0, Rotation2d.kPi);
            for (int face = 0; face < BLUE_CENTER_FACES.length; face++) {
                RED_CENTER_FACES[face] = BLUE_CENTER_FACES[face].relativeTo(RED_ORIGIN);
                BLUE_CENTER_SCORING_FACES[face] = BLUE_CENTER_FACES[face].transformBy(reefFaceScoringTransform);
                RED_CENTER_SCORING_FACES[face] = RED_CENTER_FACES[face].transformBy(reefFaceScoringTransform);

                final Map<Level, Pose3d> fillBlueRight = new HashMap<>();
                final Map<Level, Pose3d> fillRedRight = new HashMap<>();
                final Map<Level, Pose3d> fillBlueLeft = new HashMap<>();
                final Map<Level, Pose3d> fillRedLeft = new HashMap<>();

                final Map<Level, Pose2d> fillBlueScoringRight = new HashMap<>();
                final Map<Level, Pose2d> fillRedScoringRight = new HashMap<>();
                final Map<Level, Pose2d> fillBlueScoringLeft = new HashMap<>();
                final Map<Level, Pose2d> fillRedScoringLeft = new HashMap<>();

                for (final Level level : Level.values()) {
                    final Pose2d poseDirection = new Pose2d(BLUE_CENTER, Rotation2d.fromDegrees(180 - (60 * face)));
                    final double adjustX = Units.inchesToMeters(30.738);
                    final double adjustY = Units.inchesToMeters(6.469);

                    final Pose3d rightPose = new Pose3d(
                                    new Translation3d(
                                            poseDirection
                                                    .transformBy(new Transform2d(adjustX, adjustY, new Rotation2d()))
                                                    .getX(),
                                            poseDirection
                                                    .transformBy(new Transform2d(adjustX, adjustY, new Rotation2d()))
                                                    .getY(),
                                            level.heightMeters),
                                    new Rotation3d(
                                            0,
                                            Units.degreesToRadians(level.pitchDegrees),
                                            poseDirection.getRotation().getRadians())
                    );
                    final Pose2d rightScoringPose = rightPose.toPose2d().transformBy(
                            new Transform2d(
                                    Units.inchesToMeters(22),
                                    0,
                                    Rotation2d.k180deg
                            )
                    );
                    fillBlueRight.put(level, rightPose);
                    fillRedRight.put(level, rightPose.relativeTo(RED_ORIGIN_POSE3D));
                    fillBlueScoringRight.put(level, rightScoringPose);
                    fillRedScoringRight.put(level, rightScoringPose.relativeTo(RED_ORIGIN));

                    final Pose3d leftPose = new Pose3d(
                                    new Translation3d(
                                            poseDirection
                                                    .transformBy(new Transform2d(adjustX, -adjustY, new Rotation2d()))
                                                    .getX(),
                                            poseDirection
                                                    .transformBy(new Transform2d(adjustX, -adjustY, new Rotation2d()))
                                                    .getY(),
                                            level.heightMeters),
                                    new Rotation3d(
                                            0,
                                            Units.degreesToRadians(level.pitchDegrees),
                                            poseDirection.getRotation().getRadians())
                    );
                    final Pose2d leftScoringPose = leftPose.toPose2d().transformBy(
                            new Transform2d(
                                    Units.inchesToMeters(22),
                                    0,
                                    Rotation2d.k180deg
                            )
                    );
                    fillBlueLeft.put(level, leftPose);
                    fillRedLeft.put(level, leftPose.relativeTo(RED_ORIGIN_POSE3D));
                    fillBlueScoringLeft.put(level, leftScoringPose);
                    fillRedScoringLeft.put(level, leftScoringPose.relativeTo(RED_ORIGIN));
                }

                final Map<Side, Map<Level, Pose3d>> blueBranches = new HashMap<>();
                blueBranches.put(Side.LEFT, fillBlueLeft);
                blueBranches.put(Side.RIGHT, fillBlueRight);
                BLUE_BRANCH_POSITIONS.add(blueBranches);

                final Map<Side, Map<Level, Pose3d>> redBranches = new HashMap<>();
                redBranches.put(Side.LEFT, fillRedLeft);
                redBranches.put(Side.RIGHT, fillRedRight);
                RED_BRANCH_POSITIONS.add(redBranches);

                final Map<Side, Map<Level, Pose2d>> blueScoringBranches = new HashMap<>();
                blueScoringBranches.put(Side.LEFT, fillBlueScoringLeft);
                blueScoringBranches.put(Side.RIGHT, fillBlueScoringRight);
                BLUE_BRANCH_SCORING_POSITIONS.add(blueScoringBranches);

                final Map<Side, Map<Level, Pose2d>> redScoringBranches = new HashMap<>();
                redScoringBranches.put(Side.LEFT, fillRedScoringLeft);
                redScoringBranches.put(Side.RIGHT, fillRedScoringRight);
                RED_BRANCH_SCORING_POSITIONS.add(redScoringBranches);
            }
        }
    }

    private static <T> T getAllianceFlipped(final T blueAlliance, final T redAlliance) {
        return DriverStation.getAlliance()
                .map(alliance -> alliance == DriverStation.Alliance.Red ? redAlliance : blueAlliance)
                .orElse(blueAlliance);
    }

    public static Pose2d getProcessorScoringPose() {
        return getAllianceFlipped(Processor.BLUE_SCORING_POSE, Processor.RED_SCORING_POSE);
    }

    public static Pose2d[] getReefCenterPoses() {
        return getAllianceFlipped(Reef.BLUE_CENTER_FACES, Reef.RED_CENTER_FACES);
    }

    public static Pose2d[] getReefScoringCenterPoses() {
        return getAllianceFlipped(Reef.BLUE_CENTER_SCORING_FACES, Reef.RED_CENTER_SCORING_FACES);
    }

    public static List<Map<Reef.Side, Map<Reef.Level, Pose3d>>> getBranchPositions() {
        return getAllianceFlipped(Reef.BLUE_BRANCH_POSITIONS, Reef.RED_BRANCH_POSITIONS);
    }

    public static List<Map<Reef.Side, Map<Reef.Level, Pose2d>>> getBranchScoringPositions() {
        return getAllianceFlipped(Reef.BLUE_BRANCH_SCORING_POSITIONS, Reef.RED_BRANCH_SCORING_POSITIONS);
    }
}
