package frc.robot.state;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.constants.SimConstants;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.ground.intake.GroundIntake;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.superstructure.Superstructure;
import org.littletonrobotics.junction.Logger;

public class Visualizer {
    private static final String LogKey = "Visualizer";

    private final Swerve swerve;
    private final Intake intake;
    private final GroundIntake groundIntake;
    private final Superstructure superstructure;
    private final IntakeGamepieceState intakeGamepieceState;

    public Visualizer(
            final Swerve swerve,
            final Intake intake,
            final GroundIntake groundIntake,
            final Superstructure superstructure,
            final IntakeGamepieceState intakeGamepieceState
    ) {
        this.swerve = swerve;
        this.intake = intake;
        this.groundIntake = groundIntake;
        this.superstructure = superstructure;
        this.intakeGamepieceState = intakeGamepieceState;
    }

    public void periodic() {
        final Pose3d pose = new Pose3d(swerve.getPose());
        final Pose3d[] superstructurePoses = superstructure.getComponentPoses();
        final Pose3d intakeArmPose = superstructurePoses[3];
        final Pose3d groundIntakeArmPose = superstructurePoses[4];

        final Pose3d coralPose;
        if (intakeGamepieceState.hasCoral.getAsBoolean()) {
            coralPose = pose
                    .plus(new Transform3d(intakeArmPose.getTranslation(), intakeArmPose.getRotation()))
                    .plus(SimConstants.Intake.CORAL_OFFSET)
                    .plus(new Transform3d(
                            0, -intake.coralDistanceIntakeCenterMeters.getAsDouble(), 0,
                            Rotation3d.kZero
                    ));
        } else if ((intakeGamepieceState.isCoralIntaking.and(intakeGamepieceState.isGroundHandingOff)).getAsBoolean()) {
            coralPose = pose
                    .plus(new Transform3d(intakeArmPose.getTranslation(), intakeArmPose.getRotation()))
                    .plus(SimConstants.Intake.CORAL_OFFSET)
                    .plus(new Transform3d(
                            0, SimConstants.Intake.WIDTH_METERS/2,0,
                            Rotation3d.kZero
                    ));
        } else if (intakeGamepieceState.hasGroundCoral.getAsBoolean()) {
            coralPose = pose
                    .plus(new Transform3d(groundIntakeArmPose.getTranslation(), groundIntakeArmPose.getRotation()))
                    .plus(SimConstants.GroundIntakeArm.CORAL_OFFSET);
        } else {
            coralPose = Pose3d.kZero;
        }

        Logger.recordOutput(LogKey + "/CoralPose", coralPose);
    }
}
