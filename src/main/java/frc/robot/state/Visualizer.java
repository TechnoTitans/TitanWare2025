package frc.robot.state;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.constants.SimConstants;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.superstructure.Superstructure;
import org.littletonrobotics.junction.Logger;

public class Visualizer {
    private static final String LogKey = "Visualizer";

    private final Swerve swerve;
    private final Intake intake;
    private final Superstructure superstructure;
    private final GamepieceState gamepieceState;

    public Visualizer(
            final Swerve swerve,
            final Intake intake,
            final Superstructure superstructure,
            final GamepieceState gamepieceState
    ) {
        this.swerve = swerve;
        this.intake = intake;
        this.superstructure = superstructure;
        this.gamepieceState = gamepieceState;
    }

    public void periodic() {
        final Pose3d pose = new Pose3d(swerve.getPose());
        final Pose3d[] superstructurePoses = superstructure.getComponentPoses();
        final Pose3d intakeArmPose = superstructurePoses[3];

        final Pose3d coralPose;
        if (gamepieceState.hasCoral.getAsBoolean()) {
            coralPose = pose
                    .plus(new Transform3d(intakeArmPose.getTranslation(), intakeArmPose.getRotation()))
                    .plus(SimConstants.Intake.CORAL_OFFSET)
                    .plus(new Transform3d(
                            0, -intake.coralDistanceIntakeCenterMeters.getAsDouble(), 0,
                            Rotation3d.kZero
                    ));
        } else {
            coralPose = Pose3d.kZero;
        }

        Logger.recordOutput(LogKey + "/CoralPose", coralPose);
    }
}
