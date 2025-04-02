package frc.robot.subsystems.drive.controllers;

import edu.wpi.first.math.MathUtil;
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
    private static final double MinimumRotationInput = -Math.PI;
    private static final double MaxRotationInput = Math.PI;

    private final DeltaTime deltaTime;

    private final PIDController xController;
    private final PIDController yController;
    private final PIDController rotationController;

    private final TrapezoidProfile translationProfile;
    private final TrapezoidProfile rotationProfile;

    private final TrapezoidProfile.State translationUnprofiledReference;
    private final TrapezoidProfile.State rotationUnprofiledReference;

    private TrapezoidProfile.State translationPreviousProfiledReference;
    private TrapezoidProfile.State rotationPreviousProfiledReference;

    public record PositionTolerance(
            double translationToleranceMeters,
            Rotation2d rotationTolerance
    ) {}

    public record VelocityTolerance(
            double translationVelocityToleranceMeterPerSec,
            double rotationVelocityToleranceRadsPerSec
    ) {}

    private final PositionTolerance positionTolerance;
    private final VelocityTolerance velocityTolerance;

    private double distance;
    private Rotation2d heading;
    private Pose2d initialPose;
    private Pose2d desiredPose;

    /**
     * Constructs a {@link HolonomicDriveController}
     *
     * @param xController        A {@link PIDController} to respond to error in the field-relative X direction
     * @param yController        A {@link PIDController} to respond to error in the field-relative Y direction
     * @param rotationController A {@link PIDController} controller to respond to error in rotation
     */
    public HolonomicDriveController(
            final PIDController xController,
            final PIDController yController,
            final PIDController rotationController,
            final TrapezoidProfile.Constraints translationConstraints,
            final TrapezoidProfile.Constraints rotationConstraints,
            final PositionTolerance positionTolerance,
            final VelocityTolerance velocityTolerance
    ) {
        this.deltaTime = new DeltaTime();

        this.xController = xController;
        this.yController = yController;
        this.rotationController = rotationController;
        this.rotationController.enableContinuousInput(MinimumRotationInput, MaxRotationInput);

        this.translationProfile = new TrapezoidProfile(translationConstraints);
        this.translationPreviousProfiledReference = new TrapezoidProfile.State();
        this.translationUnprofiledReference = new TrapezoidProfile.State(0, 0);

        this.rotationProfile = new TrapezoidProfile(rotationConstraints);
        this.rotationPreviousProfiledReference = new TrapezoidProfile.State();
        this.rotationUnprofiledReference = new TrapezoidProfile.State(0, 0);

        this.positionTolerance = positionTolerance;
        this.velocityTolerance = velocityTolerance;

        this.distance = 0;
        this.heading = Rotation2d.kZero;
        this.initialPose = new Pose2d();
        this.desiredPose = new Pose2d();
    }

    public static Trigger atPose(
            final Supplier<Pose2d> currentPoseSupplier,
            final Supplier<Pose2d> targetPoseSupplier,
            final PositionTolerance positionTolerance
    ) {
        return new Trigger(() -> {
            final Transform2d delta = currentPoseSupplier.get().minus(targetPoseSupplier.get());

            return delta.getTranslation().getNorm() < positionTolerance.translationToleranceMeters
                    && delta.getRotation().getRadians() < positionTolerance.rotationTolerance.getRadians();
        });
    }

    public static Trigger atPoseAndStopped(
            final Supplier<Pose2d> currentPoseSupplier,
            final Supplier<ChassisSpeeds> fieldRelativeSpeedsSupplier,
            final Supplier<Pose2d> targetPoseSupplier,
            final PositionTolerance positionTolerance,
            final VelocityTolerance velocityTolerance
    ) {
        return new Trigger(() -> {
            final Transform2d delta = currentPoseSupplier.get().minus(targetPoseSupplier.get());
            final ChassisSpeeds speeds = fieldRelativeSpeedsSupplier.get();

            final double linearSpeedMetersPerSec = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);

            return delta.getTranslation().getNorm() < positionTolerance.translationToleranceMeters
                    && delta.getRotation().getRadians() < positionTolerance.rotationTolerance.getRadians()
                    && linearSpeedMetersPerSec < velocityTolerance.translationVelocityToleranceMeterPerSec
                    && Math.abs(speeds.omegaRadiansPerSecond) < velocityTolerance.rotationVelocityToleranceRadsPerSec;
        });
    }

    public Trigger atPose(
            final Supplier<Pose2d> currentPoseSupplier,
            final Supplier<Pose2d> targetPoseSupplier
    ) {
        return atPose(currentPoseSupplier, targetPoseSupplier, positionTolerance);
    }

    public Trigger atPoseAndStopped(
            final Supplier<Pose2d> currentPoseSupplier,
            final Supplier<ChassisSpeeds> fieldRelativeSpeedsSupplier,
            final Supplier<Pose2d> targetPoseSupplier
    ) {
        return atPoseAndStopped(
                currentPoseSupplier,
                fieldRelativeSpeedsSupplier,
                targetPoseSupplier,
                positionTolerance,
                velocityTolerance
        );
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

        xController.reset();
        yController.reset();
        rotationController.reset();

        final Translation2d initialToDesired = desiredPose.getTranslation().minus(currentPose.getTranslation());
        this.distance = initialToDesired.getNorm();
        this.heading = initialToDesired.getAngle();
        this.initialPose = currentPose;
        this.desiredPose = desiredPose;

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
     * @return The next output of the holonomic drive controller
     */
    public ChassisSpeeds calculate(final Pose2d currentPose) {
        final double time = deltaTime.get();

        this.translationPreviousProfiledReference = translationProfile.calculate(
                time,
                translationPreviousProfiledReference,
                translationUnprofiledReference
        );

        final Translation2d profiledTranslation = initialPose
                .getTranslation()
                .plus(new Translation2d(
                        distance - translationPreviousProfiledReference.position,
                        heading
                ));

        final double xFF = -translationPreviousProfiledReference.velocity * heading.getCos();
        final double xFeedback = xController.calculate(
                currentPose.getX(),
                profiledTranslation.getX()
        );

        final double yFF = -translationPreviousProfiledReference.velocity * heading.getSin();
        final double yFeedback = yController.calculate(
                currentPose.getY(),
                profiledTranslation.getY()
        );

        final double xSpeed = xFeedback + xFF;
        final double ySpeed = yFeedback + yFF;

        final double currentRotationRadians = currentPose.getRotation().getRadians();
        rotationUnprofiledReference.position = desiredPose.getRotation().getRadians();

        double errorBound = (MaxRotationInput - MinimumRotationInput) / 2.0;
        double goalMinDistance =
                MathUtil.inputModulus(
                        rotationUnprofiledReference.position - currentRotationRadians,
                        -errorBound,
                        errorBound
                );
        double setpointMinDistance =
                MathUtil.inputModulus(
                        rotationPreviousProfiledReference.position - currentRotationRadians,
                        -errorBound,
                        errorBound
                );

        // Recompute the profile goal with the smallest error, thus giving the shortest path. The goal
        // may be outside the input range after this operation, but that's OK because the controller
        // will still go there and report an error of zero. In other words, the setpoint only needs to
        // be offset from the measurement by the input range modulus; they don't need to be equal.
        rotationUnprofiledReference.position = goalMinDistance + currentRotationRadians;
        rotationPreviousProfiledReference.position = setpointMinDistance + currentRotationRadians;

        this.rotationPreviousProfiledReference = rotationProfile.calculate(
                time,
                rotationPreviousProfiledReference,
                rotationUnprofiledReference
        );

        final double rotationFF = rotationPreviousProfiledReference.velocity;
        final double rotationFeedback = rotationController.calculate(
                currentRotationRadians,
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
