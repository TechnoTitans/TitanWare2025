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

    private static Translation2d getElevatorTranslation(
            final double lengthMeters,
            final Rotation2d elevatorArmPosition
    ) {
        return new Translation2d(
                lengthMeters,
                elevatorArmPosition
                        .minus(ElevatorArm.ZEROED_POSITION_TO_HORIZONTAL)
        );
    }

    public static Translation2d getElevatorBaseStageTranslation(final Rotation2d elevatorArmPosition) {
        return SuperstructureSolver.getElevatorTranslation(
                Elevator.BASE_LENGTH_PIVOT_TO_PIVOT_METERS,
                elevatorArmPosition
        );
    }

    public static Translation2d getElevatorExtensionTranslation(
            final double elevatorExtensionMeters,
            final Rotation2d elevatorArmPivotPosition
    ) {
        return SuperstructureSolver.getElevatorTranslation(
                elevatorExtensionMeters,
                elevatorArmPivotPosition
        );
    }

    public static Translation2d getLowestPointOnGroundIntake(
            final Rotation2d elevatorArmPosition,
            final Rotation2d intakeArmPosition,
            final Translation2d elevatorExtensionLine
    ) {
        final Pose2d intakePose = new Pose2d(
                elevatorExtensionLine
                        .plus(new Translation2d(
                                IntakeArm.ORIGIN_OFFSET.getX(),
                                IntakeArm.ORIGIN_OFFSET.getZ()
                        )),
                Rotation2d.fromRadians(
                        elevatorArmPosition.getRadians()
                                - Rotation2d.kCCW_Pi_2.getRadians()
                                - ElevatorArm.ZEROED_POSITION_TO_HORIZONTAL.getRadians()
                                + intakeArmPosition.getRadians()
                                - IntakeArm.ZEROED_POSITION_TO_HORIZONTAL.getRadians()
                )
        );

        final Transform2d pivotToTopRoller = IntakeArm.PIVOT_TO_TOP_ROLLER;
        final Transform2d pivotToBottomRoller = IntakeArm.PIVOT_TO_BOTTOM_ROLLER;

        final Pose2d topRollerPose = intakePose.plus(pivotToTopRoller);
        final Pose2d bottomRollerPose = intakePose.plus(pivotToBottomRoller);

        final double topY = topRollerPose.getY();
        final double bottomY = bottomRollerPose.getY();
        final boolean topIsLower = topY <= bottomY;

        return topIsLower
                ? topRollerPose.getTranslation()
                : bottomRollerPose.getTranslation();
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
                        SuperstructureSolver
                                .getGroundIntakePivotAngle(groundIntakePivotPosition)
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
                SuperstructureSolver
                        .getGroundIntakePivotAngle(groundIntakePivotPosition)
                        .unaryMinus()
        );
    }

    public static Pose3d[] calculatePoses(
            final Rotation2d elevatorArmPosition,
            final double elevatorExtensionMeters,
            final Rotation2d intakeArmPosition,
            final Rotation2d groundIntakePivotPosition
    ) {
        final Pose3d baseStagePose = new Pose3d(
                ElevatorArm.ORIGIN,
                new Rotation3d(
                        0,
                        elevatorArmPosition
                                .unaryMinus()
                                .plus(Rotation2d.kCCW_Pi_2
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
                                intakeArmPosition
                                        .unaryMinus()
                                        .plus(IntakeArm.ZEROED_POSITION_TO_HORIZONTAL)
                                        .getRadians(),
                                0
                        )
                ));

        final Pose3d groundIntakePose = SuperstructureSolver
                .getGroundIntakePose(groundIntakePivotPosition);

        return new Pose3d[] {
                baseStagePose,
                stage1Pose,
                stage2Pose,
                intakePose,
                groundIntakePose
        };
    }
}
