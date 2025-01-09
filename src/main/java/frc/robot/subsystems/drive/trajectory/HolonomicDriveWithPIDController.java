package frc.robot.subsystems.drive.trajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class HolonomicDriveWithPIDController {
    private final PIDController xController;
    private final PIDController yController;
    private final ProfiledPIDController rotationController;

    /**
     * Constructs a {@link HolonomicDriveWithPIDController}
     *
     * @param xController        A {@link PIDController} to respond to error in the field-relative X direction
     * @param yController        A {@link PIDController} to respond to error in the field-relative Y direction
     * @param rotationController A {@link ProfiledPIDController} controller to respond to error in rotation
     */
    public HolonomicDriveWithPIDController(
            final PIDController xController,
            final PIDController yController,
            final ProfiledPIDController rotationController,
            final Pose2d poseTolerance
    ) {
        this.xController = xController;
        this.xController.setTolerance(poseTolerance.getX(), poseTolerance.getX() * 1.5);

        this.yController = yController;
        this.yController.setTolerance(poseTolerance.getY(), poseTolerance.getX() * 1.5);

        final Rotation2d rotationTolerance = poseTolerance.getRotation();
        this.rotationController = rotationController;
        this.rotationController.setTolerance(
                rotationTolerance.getRadians(),
                rotationTolerance.getRadians() * 1.5
        );
        this.rotationController.enableContinuousInput(-Math.PI, Math.PI);
    }

    /**
     * Resets the state of the DriveController by resetting accumulated error on all PID controllers
     *
     * @see PIDController#reset()
     * @see ProfiledPIDController#reset(double, double)
     */
    public void reset(final Pose2d currentPose, final ChassisSpeeds robotRelativeSpeeds) {
        xController.reset();
        yController.reset();
        rotationController.reset(currentPose.getRotation().getRadians(), robotRelativeSpeeds.omegaRadiansPerSecond);
    }

    /**
     * Returns true if the pose error is within tolerance of the reference.
     *
     * @return True if the pose error is within tolerance of the reference.
     */
    public boolean atReference() {
        return xController.atSetpoint()
                && yController.atSetpoint()
                && rotationController.atGoal();
    }

    public void setTolerance(final Pose2d poseTolerance) {
        this.xController.setTolerance(poseTolerance.getX(), poseTolerance.getX() * 1.5);
        this.yController.setTolerance(poseTolerance.getY(), poseTolerance.getY() * 1.5);
    }

    /**
     * Calculates the next output of the holonomic drive controller
     *
     * @param currentPose The current {@link Pose2d}
     * @param targetPose  The desired {@link Pose2d}
     * @return The next output of the holonomic drive controller
     */
    public ChassisSpeeds calculate(final Pose2d currentPose, final Pose2d targetPose) {
        final double xFeedback = xController.calculate(currentPose.getX(), targetPose.getX());
        final double yFeedback = yController.calculate(currentPose.getY(), targetPose.getY());

        final double rotationFF = rotationController.getSetpoint().velocity;
        final double rotationFeedback = rotationController.calculate(
                currentPose.getRotation().getRadians(),
                targetPose.getRotation().getRadians()
        );

        return ChassisSpeeds.fromFieldRelativeSpeeds(
                xFeedback,
                yFeedback,
                rotationFF + rotationFeedback,
                currentPose.getRotation()
        );
    }
}
