package frc.robot.subsystems.drive.controllers;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.utils.control.DeltaTime;

import java.util.function.Supplier;

public class HolonomicDriveController {
    private final DeltaTime deltaTime;

    private final PIDController translationController;
    private final PIDController rotationController;

    private final TrapezoidProfile translationProfile;
    private final TrapezoidProfile rotationProfile;

    private final TrapezoidProfile.State translationUnprofiledReference;
    private final TrapezoidProfile.State rotationUnprofiledReference;

    private TrapezoidProfile.State translationPreviousProfiledReference;
    private TrapezoidProfile.State rotationPreviousProfiledReference;

    public record Tolerance(double translationToleranceMeters, Rotation2d rotationTolerance) {}
    private final Tolerance tolerance;

    /**
     * Constructs a {@link HolonomicDriveController}
     *
     * @param translationController        A {@link PIDController} to respond to error in the field-relative X direction
     * @param rotationController A {@link PIDController} controller to respond to error in rotation
     */
    public HolonomicDriveController(
            final PIDController translationController,
            final PIDController rotationController,
            final TrapezoidProfile.Constraints translationConstraints,
            final TrapezoidProfile.Constraints rotationConstraints,
            final Tolerance tolerance
    ) {
        this.deltaTime = new DeltaTime();

        this.translationController = translationController;
        this.rotationController = rotationController;
        this.rotationController.enableContinuousInput(-Math.PI, Math.PI);

        this.translationProfile = new TrapezoidProfile(translationConstraints);
        this.translationPreviousProfiledReference = new TrapezoidProfile.State();
        this.translationUnprofiledReference = new TrapezoidProfile.State(0, 0);

        this.rotationProfile = new TrapezoidProfile(rotationConstraints);
        this.rotationPreviousProfiledReference = new TrapezoidProfile.State();
        this.rotationUnprofiledReference = new TrapezoidProfile.State(0, 0);

        this.tolerance = tolerance;
    }

    public Trigger atPose(
            final Supplier<Pose2d> currentPoseSupplier,
            final Supplier<Pose2d> targetPoseSupplier
    ) {
        return new Trigger(() -> {
            final Transform2d delta = currentPoseSupplier.get().minus(targetPoseSupplier.get());
            return delta.getTranslation().getNorm() < tolerance.translationToleranceMeters
                    && delta.getRotation().getRadians() < tolerance.rotationTolerance.getRadians();
        });
    }

    /**
     * Resets the state of the DriveController by resetting accumulated error on all PID controllers
     *
     * @see PIDController#reset()
     */
    public void reset(
            final Pose2d currentPose,
            final Pose2d desiredPose,
            final ChassisSpeeds fieldRelativeSpeeds
    ) {
        deltaTime.reset();
        translationController.reset();
        rotationController.reset();

        final Translation2d fieldSpeeds = new Translation2d(
                fieldRelativeSpeeds.vxMetersPerSecond,
                fieldRelativeSpeeds.vyMetersPerSecond
        );

        this.translationPreviousProfiledReference = new TrapezoidProfile.State(
                currentPose.getTranslation().getDistance(desiredPose.getTranslation()),
                Math.min(0, -fieldSpeeds
                        .rotateBy(
                                desiredPose
                                        .getTranslation()
                                        .minus(currentPose.getTranslation())
                                        .getAngle()
                                        .unaryMinus()
                        ).getX())
        );

        this.rotationPreviousProfiledReference = new TrapezoidProfile.State(
                currentPose.getRotation().getRadians(), fieldRelativeSpeeds.omegaRadiansPerSecond
        );
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

        final Rotation2d rotationDifference =
                currentPose
                        .getTranslation()
                        .minus(targetPose.getTranslation())
                        .getAngle();

        this.translationPreviousProfiledReference = translationProfile.calculate(
                time,
                translationPreviousProfiledReference,
                translationUnprofiledReference
        );

        final double translationFF = translationPreviousProfiledReference.velocity;
        final double translationFeedback = translationController.calculate(
                targetPose.getTranslation().getDistance(currentPose.getTranslation()),
                translationPreviousProfiledReference.position
        );

        final double translationSpeed = translationFeedback + translationFF;
        final double xSpeed = translationSpeed * Math.cos(rotationDifference.getRadians());
        final double ySpeed = translationSpeed * Math.sin(rotationDifference.getRadians());

        this.rotationUnprofiledReference.position = targetPose.getRotation().getRadians();
        this.rotationPreviousProfiledReference = rotationProfile.calculate(
                time,
                rotationPreviousProfiledReference,
                rotationUnprofiledReference
        );
        final double rotationFF = rotationPreviousProfiledReference.velocity;
        final double rotationFeedback = rotationController.calculate(
                currentPose.getRotation().getRadians(),
                rotationPreviousProfiledReference.position
        );
        final double rotationSpeed = rotationFeedback + rotationFF;

        return ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed,
                ySpeed,
                rotationSpeed,
                currentPose.getRotation()
        );
    }
}
