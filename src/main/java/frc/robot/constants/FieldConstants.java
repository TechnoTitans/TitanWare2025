package frc.robot.constants;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;

public class FieldConstants {
    //borrowed from mechanical advantage :)
    public static final double FIELD_LENGTH_X_METERS = Units.inchesToMeters(690.876);
    public static final double FIELD_WIDTH_Y_METERS = Units.inchesToMeters(317);
    public static final Pose2d RED_ORIGIN = new Pose2d(FIELD_LENGTH_X_METERS, FIELD_WIDTH_Y_METERS, Rotation2d.k180deg);

    public static final Transform2d SCORING_DISTANCE_OFFSET =
            new Transform2d(Units.inchesToMeters(25.5), 0, Rotation2d.kPi);
    public static final Transform2d AUTO_SCORING_DISTANCE_OFFSET =
            new Transform2d(Units.inchesToMeters(7.5), 0, Rotation2d.kZero);
    public static final Transform2d ALGAE_DESCORE_DISTANCE_OFFSET =
            new Transform2d(Units.inchesToMeters(25), 0, Rotation2d.kPi);
    public static final Transform2d ALGAE_SAFE_REEF_OFFSET =
            new Transform2d(-Units.inchesToMeters(12), 0, Rotation2d.kZero);
    public static final Transform2d PROCESSOR_DISTANCE_OFFSET =
            new Transform2d(Units.inchesToMeters(32), 0, Rotation2d.kPi);
    public static final Transform2d ALIGN_PROCESSOR_DISTANCE_OFFSET =
            new Transform2d(Units.inchesToMeters(-24), 0, Rotation2d.kZero);
    public static final Transform2d HP_DISTANCE_OFFSET =
            new Transform2d(Units.inchesToMeters(17.5), 0, Rotation2d.kPi);
    //TODO: Maybe move more back because it might hit barge tag
    public static final Transform2d SCORING_BARGE_OFFSET =
            new Transform2d(-Units.inchesToMeters(36.5), 0, Rotation2d.kPi);

    public static class Processor {
        public static final Pose2d BLUE_CENTER_FACE =
                new Pose2d(Units.inchesToMeters(235.726), 0, Rotation2d.fromDegrees(90));
        public static final Pose2d RED_CENTER_FACE = BLUE_CENTER_FACE.relativeTo(RED_ORIGIN);

        public static final Pose2d BLUE_SCORING_POSE = BLUE_CENTER_FACE.transformBy(PROCESSOR_DISTANCE_OFFSET);
        public static final Pose2d RED_SCORING_POSE = RED_CENTER_FACE.transformBy(PROCESSOR_DISTANCE_OFFSET);

        public static final Pose2d BLUE_ALIGN_POSE = BLUE_SCORING_POSE.transformBy(ALIGN_PROCESSOR_DISTANCE_OFFSET);
        public static final Pose2d RED_ALIGN_POSE = RED_SCORING_POSE.transformBy(ALIGN_PROCESSOR_DISTANCE_OFFSET);
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

        public static final Pose2d NET_SCORING_BLUE_LEFT = BLUE_LEFT_CAGE.transformBy(SCORING_BARGE_OFFSET);
        public static final Pose2d NET_SCORING_BLUE_MIDDLE = BLUE_MIDDLE_CAGE.transformBy(SCORING_BARGE_OFFSET);
        public static final Pose2d NET_SCORING_BLUE_RIGHT = BLUE_RIGHT_CAGE.transformBy(SCORING_BARGE_OFFSET);
        public static final Pose2d NET_SCORING_RED_LEFT = RED_LEFT_CAGE.transformBy(SCORING_BARGE_OFFSET);
        public static final Pose2d NET_SCORING_RED_MIDDLE = RED_MIDDLE_CAGE.transformBy(SCORING_BARGE_OFFSET);
        public static final Pose2d NET_SCORING_RED_RIGHT = RED_RIGHT_CAGE.transformBy(SCORING_BARGE_OFFSET);
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
        public static final List<Pose2d> BLUE_PICKUP_CORAL_STATIONS_POSES = List.of(
                BLUE_LEFT_CENTER_FACE.transformBy(HP_DISTANCE_OFFSET),
                BLUE_RIGHT_CENTER_FACE.transformBy(HP_DISTANCE_OFFSET)
        );

        public static final Pose2d RED_LEFT_CENTER_FACE = BLUE_LEFT_CENTER_FACE.relativeTo(RED_ORIGIN);
        public static final Pose2d RED_RIGHT_CENTER_FACE = BLUE_RIGHT_CENTER_FACE.relativeTo(RED_ORIGIN);
        public static final List<Pose2d> RED_CORAL_STATIONS = List.of(
                RED_LEFT_CENTER_FACE,
                RED_RIGHT_CENTER_FACE
        );
        public static final List<Pose2d> RED_PICKUP_CORAL_STATIONS_POSES = List.of(
                RED_LEFT_CENTER_FACE.transformBy(HP_DISTANCE_OFFSET),
                RED_RIGHT_CENTER_FACE.transformBy(HP_DISTANCE_OFFSET)
        );
    }

    public static class Reef {
        public enum Face {
            ZERO_FACING_DRIVERS,
            ONE,
            TWO,
            THREE_FACING_CLIMB,
            FOUR,
            FIVE
        }

        public enum Side {
            RIGHT,
            LEFT
        }

        public enum Level {
            AUTO_L4(Units.inchesToMeters(72), -90, Transform2d.kZero),
            L4(Units.inchesToMeters(72), -90, Transform2d.kZero),
            L3(Units.inchesToMeters(47.625), -35, Transform2d.kZero),
            L2(Units.inchesToMeters(31.875), -35, Transform2d.kZero),
            L1(Units.inchesToMeters(18), 0, new Transform2d(-Units.inchesToMeters(2), 0, Rotation2d.kZero));

            public final double heightMeters;
            public final double pitchDegrees;
            public final Transform2d scoringOffset;

            Level(final double heightMeters, final double pitchDegrees, final Transform2d scoringOffset) {
                this.heightMeters = heightMeters;
                this.pitchDegrees = pitchDegrees;
                this.scoringOffset = scoringOffset;
            }
        }

        public static final Set<Integer> BLUE_APRILTAG_IDS = Set.of(18, 19, 20, 21, 22, 17);
        public static final Set<Integer> RED_APRILTAG_IDS = Set.of(7, 6, 11, 10, 9, 8);

        public static final Translation2d BLUE_CENTER =
                new Translation2d(Units.inchesToMeters(176.746), Units.inchesToMeters(158.501));
        public static final Translation2d RED_CENTER = BLUE_CENTER.plus(
                new Translation2d(-FIELD_LENGTH_X_METERS, 0)
        );

        // Side of the reef to the inside of the reef zone line
        public static final double faceToZoneLine = Units.inchesToMeters(12);

        // Starting facing the driver station in clockwise order
        public static final Map<Face, Pose2d> BLUE_CENTER_FACES = new HashMap<>();
        public static final Map<Face, Pose2d> BLUE_CENTER_SCORING_FACES = new HashMap<>();
        public static final Map<Face, Pose2d> RED_CENTER_FACES = new HashMap<>();
        public static final Map<Face, Pose2d> RED_CENTER_SCORING_FACES = new HashMap<>();
        // Starting at the right branch facing the driver station in clockwise
        public static final Map<Face, Map<Side, Map<Level, Pose3d>>> BLUE_BRANCH_POSITIONS = new HashMap<>();
        public static final Map<Face, Map<Side, Map<Level, Pose3d>>> RED_BRANCH_POSITIONS = new HashMap<>();
        public static final Map<Face, Map<Side, Map<Level, Pose2d>>> BLUE_BRANCH_SCORING_POSITIONS = new HashMap<>();
        public static final Map<Face, Map<Side, Map<Level, Pose2d>>> RED_BRANCH_SCORING_POSITIONS = new HashMap<>();

        static {
            BLUE_CENTER_FACES.put(
                    Face.ZERO_FACING_DRIVERS,
                    new Pose2d(
                            Units.inchesToMeters(144.003),
                            Units.inchesToMeters(158.500),
                            Rotation2d.fromDegrees(180))
            );

            BLUE_CENTER_FACES.put(
                    Face.ONE,
                    new Pose2d(
                            Units.inchesToMeters(160.373),
                            Units.inchesToMeters(186.857),
                            Rotation2d.fromDegrees(120))
            );

            BLUE_CENTER_FACES.put(
                    Face.TWO,
                    new Pose2d(
                            Units.inchesToMeters(193.116),
                            Units.inchesToMeters(186.858),
                            Rotation2d.fromDegrees(60))
            );

            BLUE_CENTER_FACES.put(
                    Face.THREE_FACING_CLIMB,
                    new Pose2d(
                            Units.inchesToMeters(209.489),
                            Units.inchesToMeters(158.502),
                            Rotation2d.fromDegrees(0))
            );

            BLUE_CENTER_FACES.put(
                    Face.FOUR,
                    new Pose2d(
                            Units.inchesToMeters(193.118),
                            Units.inchesToMeters(130.145),
                            Rotation2d.fromDegrees(-60))
            );

            BLUE_CENTER_FACES.put(
                    Face.FIVE,
                    new Pose2d(
                            Units.inchesToMeters(160.375),
                            Units.inchesToMeters(130.144),
                            Rotation2d.fromDegrees(-120))
            );

            final Pose3d RED_ORIGIN_POSE3D = new Pose3d(RED_ORIGIN);;
            for (final Map.Entry<Face, Pose2d> entry : BLUE_CENTER_FACES.entrySet()) {
                RED_CENTER_FACES.put(entry.getKey(), entry.getValue().relativeTo(RED_ORIGIN));
                BLUE_CENTER_SCORING_FACES.put(entry.getKey(), entry.getValue().transformBy(SCORING_DISTANCE_OFFSET));
                RED_CENTER_SCORING_FACES.put(
                        entry.getKey(),
                        RED_CENTER_FACES.get(entry.getKey()).transformBy(SCORING_DISTANCE_OFFSET)
                );

                final Map<Level, Pose3d> fillBlueRight = new HashMap<>();
                final Map<Level, Pose3d> fillRedRight = new HashMap<>();
                final Map<Level, Pose3d> fillBlueLeft = new HashMap<>();
                final Map<Level, Pose3d> fillRedLeft = new HashMap<>();

                final Map<Level, Pose2d> fillBlueScoringRight = new HashMap<>();
                final Map<Level, Pose2d> fillRedScoringRight = new HashMap<>();
                final Map<Level, Pose2d> fillBlueScoringLeft = new HashMap<>();
                final Map<Level, Pose2d> fillRedScoringLeft = new HashMap<>();

                for (final Level level : Level.values()) {
                    final Pose2d poseDirection = new Pose2d(
                            BLUE_CENTER,
                            Rotation2d.fromDegrees(180 - (60 * entry.getKey().ordinal()))
                    );
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
                    final Pose2d rightScoringPose = rightPose.toPose2d()
                            .transformBy(SCORING_DISTANCE_OFFSET)
                            .transformBy(level.scoringOffset);

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
                    final Pose2d leftScoringPose = leftPose.toPose2d()
                            .transformBy(SCORING_DISTANCE_OFFSET)
                            .transformBy(level.scoringOffset);

                    fillBlueLeft.put(level, leftPose);
                    fillRedLeft.put(level, leftPose.relativeTo(RED_ORIGIN_POSE3D));
                    fillBlueScoringLeft.put(level, leftScoringPose);
                    fillRedScoringLeft.put(level, leftScoringPose.relativeTo(RED_ORIGIN));
                }

                final Map<Side, Map<Level, Pose3d>> blueBranches = new HashMap<>();
                blueBranches.put(Side.LEFT, fillBlueLeft);
                blueBranches.put(Side.RIGHT, fillBlueRight);
                BLUE_BRANCH_POSITIONS.put(entry.getKey(), blueBranches);

                final Map<Side, Map<Level, Pose3d>> redBranches = new HashMap<>();
                redBranches.put(Side.LEFT, fillRedLeft);
                redBranches.put(Side.RIGHT, fillRedRight);
                RED_BRANCH_POSITIONS.put(entry.getKey(), redBranches);

                final Map<Side, Map<Level, Pose2d>> blueScoringBranches = new HashMap<>();
                blueScoringBranches.put(Side.LEFT, fillBlueScoringLeft);
                blueScoringBranches.put(Side.RIGHT, fillBlueScoringRight);
                BLUE_BRANCH_SCORING_POSITIONS.put(entry.getKey(), blueScoringBranches);

                final Map<Side, Map<Level, Pose2d>> redScoringBranches = new HashMap<>();
                redScoringBranches.put(Side.LEFT, fillRedScoringLeft);
                redScoringBranches.put(Side.RIGHT, fillRedScoringRight);
                RED_BRANCH_SCORING_POSITIONS.put(entry.getKey(), redScoringBranches);
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

    public static Pose2d getProcessorAlignPose() {
        return getAllianceFlipped(Processor.BLUE_ALIGN_POSE, Processor.RED_ALIGN_POSE);
    }

    public static List<Pose2d> getHPPickupPoses() {
        return getAllianceFlipped(
                CoralStation.BLUE_PICKUP_CORAL_STATIONS_POSES, CoralStation.RED_PICKUP_CORAL_STATIONS_POSES
        );
    }

    public static Map<Reef.Face, Pose2d> getReefCenterPoses() {
        return getAllianceFlipped(Reef.BLUE_CENTER_FACES, Reef.RED_CENTER_FACES);
    }

    public static Map<Reef.Face, Pose2d> getReefScoringCenterPoses() {
        return getAllianceFlipped(Reef.BLUE_CENTER_SCORING_FACES, Reef.RED_CENTER_SCORING_FACES);
    }

    public static Pose2d getCenterCage() {
        return getAllianceFlipped(Barge.BLUE_MIDDLE_CAGE, Barge.RED_MIDDLE_CAGE);
    }

    public static Pose2d getScoringBargeCenterCage() {
        return getAllianceFlipped(Barge.NET_SCORING_BLUE_MIDDLE, Barge.NET_SCORING_RED_MIDDLE);
    }

    public static Map<Reef.Face, Map<Reef.Side, Map<Reef.Level, Pose3d>>> getBranchPositions() {
        return getAllianceFlipped(Reef.BLUE_BRANCH_POSITIONS, Reef.RED_BRANCH_POSITIONS);
    }

    public static Map<Reef.Face, Map<Reef.Side, Map<Reef.Level, Pose2d>>> getBranchScoringPositions() {
        return getAllianceFlipped(Reef.BLUE_BRANCH_SCORING_POSITIONS, Reef.RED_BRANCH_SCORING_POSITIONS);
    }
}
