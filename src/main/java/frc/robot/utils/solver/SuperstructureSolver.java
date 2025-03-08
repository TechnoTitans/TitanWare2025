package frc.robot.utils.solver;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.constants.SimConstants;
import frc.robot.constants.SimConstants.Elevator;

public class SuperstructureSolver {
    private SuperstructureSolver() {}

    public static Pose3d[] calculatePoses(
            final Rotation2d baseStageRotation,
            final double elevatorExtensionMeters,
            final Rotation2d armPivotRotation
    ) {
        final Pose3d baseStagePose = new Pose3d(
                Elevator.ORIGIN,
                new Rotation3d(
                        0,
                        baseStageRotation
                                .unaryMinus()
                                .plus(
                                        Rotation2d.kCCW_Pi_2
                                                .plus(SimConstants.ElevatorArm.ZEROED_POSITION_TO_HORIZONTAL)
                                ).getRadians(),
                        0
                )
        );

        final double stage1ExtensionMeters = Math.min(elevatorExtensionMeters, Elevator.STAGE_1_MAX_EXTENSION_METERS);
        final Pose3d stage1Pose = baseStagePose.transformBy(
                new Transform3d(
                        0,
                        0,
                        stage1ExtensionMeters,
                        Rotation3d.kZero
                )
        );

        final double stage2ExtensionMeters = Math.min(elevatorExtensionMeters - stage1ExtensionMeters, Elevator.STAGE_2_MAX_EXTENSION_METERS);
        final Pose3d stage2Pose = stage1Pose.transformBy(
                new Transform3d(
                        0,
                        0,
                        stage2ExtensionMeters,
                        Rotation3d.kZero
                )
        );

        final Pose3d intakePose = stage2Pose.transformBy(
                new Transform3d(
                        0,
                        0,
                        Elevator.STAGE_2_TO_INTAKE,
                        new Rotation3d(
                                0,
                                armPivotRotation
                                        .unaryMinus()
                                        .minus(SimConstants.IntakeArm.ZEROED_POSITION_TO_HORIZONTAL)
                                        .getRadians(),
                                0
                        )
                )
        );

        return new Pose3d[] {baseStagePose, stage1Pose, stage2Pose, intakePose};
    }
}
