package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.geometry.*;
import frc.robot.constants.SimConstants.Elevator;
import frc.robot.constants.SimConstants.ElevatorArm;
import frc.robot.constants.SimConstants.GroundIntakeArm;
import frc.robot.constants.SimConstants.IntakeArm;

public class SuperstructureSolver {
    private SuperstructureSolver() {}

    public static Translation2d getElevatorArmPivotOrigin2d() {
        final Translation3d origin3d = ElevatorArm.ORIGIN;
        return new Translation2d(
                origin3d.getX(),
                origin3d.getZ()
        );
    }

    public static Translation2d getElevatorBaseStageTranslation(final Rotation2d elevatorArmPosition) {
        return new Translation2d(
                Elevator.BASE_LENGTH_METERS,
                elevatorArmPosition
                        .minus(ElevatorArm.ZEROED_POSITION_TO_HORIZONTAL)
        );
    }

    private static Rotation2d getGroundIntakePivotAngle(final Rotation2d groundIntakePivotPosition) {
        return GroundIntakeArm.ZEROED_POSITION_TO_HORIZONTAL
                .minus(groundIntakePivotPosition);
    }

    private static Pose3d getGroundIntakePose(final Rotation2d groundIntakePivotPosition) {
        return new Pose3d(
                GroundIntakeArm.ORIGIN,
                new Rotation3d(
                        0,
                        getGroundIntakePivotAngle(groundIntakePivotPosition)
                                .getRadians(),
                        0
                )
        );
    }

    public static Pose2d getGroundIntakePivotPose2d(final Rotation2d groundIntakePivotPosition) {
        final Pose3d groundIntakePose = getGroundIntakePose(groundIntakePivotPosition);
        return new Pose2d(
                groundIntakePose.getX(),
                groundIntakePose.getZ(),
                getGroundIntakePivotAngle(groundIntakePivotPosition)
                        .unaryMinus()
        );
    }

    public static Pose3d[] calculatePoses(
            final Rotation2d elevatorArmPosition,
            final double elevatorExtensionMeters,
            final Rotation2d intakePivotPosition,
            final Rotation2d groundIntakePivotPosition
    ) {
        final Pose3d baseStagePose = new Pose3d(
                ElevatorArm.ORIGIN,
                new Rotation3d(
                        0,
                        elevatorArmPosition
                                .unaryMinus()
                                .plus(
                                        Rotation2d.kCCW_Pi_2
                                                .plus(ElevatorArm.ZEROED_POSITION_TO_HORIZONTAL)
                                ).getRadians(),
                        0
                )
        );

        final double stage1ExtensionMeters = Math.min(elevatorExtensionMeters, Elevator.STAGE_1_MAX_EXTENSION_METERS);
        final Pose3d stage1Pose = baseStagePose.transformBy(
                new Transform3d(
                        0,
                        0,
                        Elevator.ORIGIN_OFFSET
                                + stage1ExtensionMeters,
                        Rotation3d.kZero
                )
        );

        final double stage2ExtensionMeters = Math.min(elevatorExtensionMeters - stage1ExtensionMeters, Elevator.STAGE_2_MAX_EXTENSION_METERS);
        final Pose3d stage2Pose = stage1Pose.transformBy(
                new Transform3d(
                        0,
                        0,
                        Elevator.STAGE_2_OFFSET
                                + stage2ExtensionMeters,
                        Rotation3d.kZero
                )
        );

        final Pose3d intakePose = stage2Pose
                .transformBy(IntakeArm.ORIGIN_OFFSET)
                .transformBy(new Transform3d(
                        0,
                        0,
                        Elevator.STAGE_2_TO_INTAKE,
                        new Rotation3d(
                                0,
                                intakePivotPosition
                                        .unaryMinus()
                                        .plus(IntakeArm.ZEROED_POSITION_TO_HORIZONTAL)
                                        .getRadians(),
                                0
                        )
                ));

        final Pose3d groundIntakePose = getGroundIntakePose(groundIntakePivotPosition);

        return new Pose3d[] {
                baseStagePose,
                stage1Pose,
                stage2Pose,
                intakePose,
                groundIntakePose
        };
    }
}
