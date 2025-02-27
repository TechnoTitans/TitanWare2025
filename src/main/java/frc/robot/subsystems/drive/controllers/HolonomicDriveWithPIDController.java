package frc.robot.subsystems.drive.controllers;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.utils.control.DeltaTime;

public class HolonomicDriveWithPIDController {
    private final DeltaTime deltaTime;

    private final PIDController xController;
    private final PIDController yController;
    private final TrapezoidProfile xTrapezoidProfile;
    private final TrapezoidProfile yTrapezoidProfile;
    private final PIDController rotationController;

    private TrapezoidProfile.State xPreviousProfiledReference;
    private TrapezoidProfile.State yPreviousProfiledReference;

    private TrapezoidProfile.State xUnprofiledReference;
    private TrapezoidProfile.State yUnprofiledReference;

    /**
     * Constructs a {@link HolonomicDriveWithPIDController}
     *
     * @param xController        A {@link PIDController} to respond to error in the field-relative X direction
     * @param yController        A {@link PIDController} to respond to error in the field-relative Y direction
     * @param rotationController A {@link PIDController} controller to respond to error in rotation
     */
    public HolonomicDriveWithPIDController(
            final PIDController xController,
            final PIDController yController,
            final TrapezoidProfile.Constraints xyConstraints,
            final PIDController rotationController,
            final Pose2d poseTolerance
    ) {
        this.deltaTime = new DeltaTime();

        this.xController = xController;
        this.xController.setTolerance(poseTolerance.getX(), poseTolerance.getX() * 1.5);

        this.yController = yController;
        this.yController.setTolerance(poseTolerance.getY(), poseTolerance.getX() * 1.5);

        this.xTrapezoidProfile = new TrapezoidProfile(xyConstraints);
        this.yTrapezoidProfile = new TrapezoidProfile(xyConstraints);
        this.xPreviousProfiledReference = new TrapezoidProfile.State();
        this.yPreviousProfiledReference = new TrapezoidProfile.State();
        this.xUnprofiledReference = new TrapezoidProfile.State();
        this.yUnprofiledReference = new TrapezoidProfile.State();

        this.rotationController = rotationController;
        this.rotationController.enableContinuousInput(-Math.PI, Math.PI);
    }

    /**
     * Resets the state of the DriveController by resetting accumulated error on all PID controllers
     *
     * @see PIDController#reset()
     * @see ProfiledPIDController#reset(double, double)
     */
    public void reset(final Pose2d currentPose, final ChassisSpeeds fieldRelativeSpeeds) {
        xController.reset();
        yController.reset();
        rotationController.reset();
        deltaTime.reset();

        this.xPreviousProfiledReference = new TrapezoidProfile.State(
                currentPose.getX(), fieldRelativeSpeeds.vxMetersPerSecond
        );
        this.yPreviousProfiledReference = new TrapezoidProfile.State(
                currentPose.getY(), fieldRelativeSpeeds.vyMetersPerSecond
        );
    }

    /**
     * Returns true if the pose error is within tolerance of the reference.
     *
     * @return True if the pose error is within tolerance of the reference.
     */
    public boolean atReference() {
        return xController.atSetpoint()
                && yController.atSetpoint()
                && rotationController.atSetpoint();
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
        final double time = deltaTime.get();

        this.xUnprofiledReference.position = targetPose.getX();
        this.xPreviousProfiledReference = xTrapezoidProfile.calculate(
                time,
                xPreviousProfiledReference,
                xUnprofiledReference
        );
        final double xFeedback = xController.calculate(currentPose.getX(), xPreviousProfiledReference.position);

        this.yUnprofiledReference.position = targetPose.getY();
        this.yPreviousProfiledReference = yTrapezoidProfile.calculate(
                time,
                yPreviousProfiledReference,
                yUnprofiledReference
        );
        final double yFeedback = yController.calculate(currentPose.getY(), yPreviousProfiledReference.position);

        final double rotationFeedback = rotationController.calculate(
                currentPose.getRotation().getRadians(),
                targetPose.getRotation().getRadians()
        );

        return ChassisSpeeds.fromFieldRelativeSpeeds(
                xFeedback,
                yFeedback,
                rotationFeedback,
                currentPose.getRotation()
        );
    }
}
