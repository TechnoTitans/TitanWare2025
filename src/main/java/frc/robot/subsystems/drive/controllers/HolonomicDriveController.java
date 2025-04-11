package frc.robot.subsystems.drive.controllers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.Constants;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

public class HolonomicDriveController {
    private static final String LogKey = "HolonomicDriveController";

    private static final double AllowableDistanceMetersBeforeReset = 0.3;
    private static final double AllowableAngleRadsBeforeReset = Units.degreesToRadians(15);
    private static final double AllowableHeadingRadsBeforeReset = Units.degreesToRadians(15);

    private static final double TranslationFFMaxRadiusMeters = 0.05;
    private static final double TranslationFFMinRadiusMeters = 0.01;

    private static final double MinimumRotationInput = -Math.PI;
    private static final double MaxRotationInput = Math.PI;

    private final PIDController xController;
    private final PIDController yController;
    private final PIDController rotationController;

    private final TrapezoidProfile translationProfile;
    private final TrapezoidProfile rotationProfile;

    private final TrapezoidProfile.State translationGoal;
    private final TrapezoidProfile.State rotationGoal;

    private TrapezoidProfile.State translationSetpoint;
    private TrapezoidProfile.State rotationSetpoint;

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
    private final Supplier<ChassisSpeeds> fieldRelativeSpeedsSupplier;

    private Translation2d lastSetpointTranslation;

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
            final Supplier<ChassisSpeeds> fieldRelativeSpeedsSupplier,
            final PositionTolerance positionTolerance,
            final VelocityTolerance velocityTolerance
    ) {
        this.xController = xController;
        this.yController = yController;
        this.rotationController = rotationController;
        this.rotationController.enableContinuousInput(MinimumRotationInput, MaxRotationInput);

        this.translationProfile = new TrapezoidProfile(translationConstraints);
        this.translationSetpoint = new TrapezoidProfile.State();
        this.translationGoal = new TrapezoidProfile.State(0, 0);

        this.rotationProfile = new TrapezoidProfile(rotationConstraints);
        this.rotationSetpoint = new TrapezoidProfile.State();
        this.rotationGoal = new TrapezoidProfile.State(0, 0);

        this.positionTolerance = positionTolerance;
        this.velocityTolerance = velocityTolerance;
        this.fieldRelativeSpeedsSupplier = fieldRelativeSpeedsSupplier;

        this.lastSetpointTranslation = Translation2d.kZero;
    }

    public static Trigger atPose(
            final Supplier<Pose2d> currentPoseSupplier,
            final Supplier<Pose2d> targetPoseSupplier,
            final PositionTolerance positionTolerance
    ) {
        return new Trigger(() -> {
            final Transform2d delta = currentPoseSupplier.get().minus(targetPoseSupplier.get());

            final double translationDistanceMeters = Math.abs(delta.getTranslation().getNorm());
            final double rotationDeltaRads = Math.abs(MathUtil.angleModulus(delta.getRotation().getRadians()));

            return translationDistanceMeters < positionTolerance.translationToleranceMeters
                    && rotationDeltaRads < positionTolerance.rotationTolerance.getRadians();
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

            final double translationDistanceMeters = Math.abs(delta.getTranslation().getNorm());
            final double rotationDeltaRads = Math.abs(MathUtil.angleModulus(delta.getRotation().getRadians()));

            final double linearSpeedMetersPerSec = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);

            return translationDistanceMeters < positionTolerance.translationToleranceMeters
                    && rotationDeltaRads < positionTolerance.rotationTolerance.getRadians()
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
            final Pose2d targetPose
    ) {
        final ChassisSpeeds fieldSpeeds = fieldRelativeSpeedsSupplier.get();
        final Translation2d linearFieldSpeeds = new Translation2d(
                fieldSpeeds.vxMetersPerSecond,
                fieldSpeeds.vyMetersPerSecond
        );

        xController.reset();
        yController.reset();
        rotationController.reset();

        this.translationSetpoint = new TrapezoidProfile.State(
                currentPose.getTranslation().getDistance(targetPose.getTranslation()),
                Math.min(0, -linearFieldSpeeds
                        .rotateBy(
                                targetPose
                                        .getTranslation()
                                        .minus(currentPose.getTranslation())
                                        .getAngle()
                                        .unaryMinus()
                        ).getX())
        );

        this.rotationSetpoint = new TrapezoidProfile.State(
                currentPose.getRotation().getRadians(),
                fieldSpeeds.omegaRadiansPerSecond
        );

        this.lastSetpointTranslation = currentPose.getTranslation();
    }

    /**
     * Calculates the next output of the holonomic drive controller
     *
     * @param currentPose The current {@link Pose2d}
     * @param targetPose The target {@link Pose2d}
     * @return The next output of the holonomic drive controller
     */
    public ChassisSpeeds calculate(final Pose2d currentPose, final Pose2d targetPose) {
        final Translation2d targetToCurrent = currentPose.getTranslation()
                .minus(targetPose.getTranslation());
        final double currentDistance = targetToCurrent.getNorm();
        final Rotation2d currentAngle = targetToCurrent.getAngle();

        this.translationSetpoint = translationProfile.calculate(
                Constants.LOOP_PERIOD_SECONDS,
                translationSetpoint,
                translationGoal
        );

        final double setpointVelocity = translationSetpoint.velocity;
        this.lastSetpointTranslation = targetPose
                .getTranslation()
                .plus(new Translation2d(
                        translationSetpoint.position,
                        currentAngle
                ));

        final Translation2d targetToSetpoint = lastSetpointTranslation
                .minus(targetPose.getTranslation());
        final double setpointDistance = targetToSetpoint.getNorm();
        final Rotation2d setpointAngle = Math.abs(setpointDistance) < 1e-6
                ? Rotation2d.kZero
                : targetToSetpoint.getAngle();

        final double translationFFScalar = MathUtil.clamp(
                (currentDistance - TranslationFFMinRadiusMeters)
                        / (TranslationFFMaxRadiusMeters - TranslationFFMinRadiusMeters),
                0.0,
                1.0
        );
        final double translationFF = setpointVelocity * translationFFScalar;

        final double xFF = translationFF * currentAngle.getCos();
        final double xFeedback = xController.calculate(
                currentPose.getX(),
                lastSetpointTranslation.getX()
        );

        final double yFF = translationFF * currentAngle.getSin();
        final double yFeedback = yController.calculate(
                currentPose.getY(),
                lastSetpointTranslation.getY()
        );

        final double xSpeed = xFeedback + xFF;
        final double ySpeed = yFeedback + yFF;

        final double currentRotationRads = currentPose.getRotation().getRadians();
        rotationGoal.position = targetPose.getRotation().getRadians();

        double errorBound = (MaxRotationInput - MinimumRotationInput) / 2.0;
        double goalMinDistance =
                MathUtil.inputModulus(
                        rotationGoal.position - currentRotationRads,
                        -errorBound,
                        errorBound
                );
        double setpointMinDistance =
                MathUtil.inputModulus(
                        rotationSetpoint.position - currentRotationRads,
                        -errorBound,
                        errorBound
                );

        // Recompute the profile goal with the smallest error, thus giving the shortest path. The goal
        // may be outside the input range after this operation, but that's OK because the controller
        // will still go there and report an error of zero. In other words, the setpoint only needs to
        // be offset from the measurement by the input range modulus; they don't need to be equal.
        rotationGoal.position = goalMinDistance + currentRotationRads;
        rotationSetpoint.position = setpointMinDistance + currentRotationRads;

        this.rotationSetpoint = rotationProfile.calculate(
                Constants.LOOP_PERIOD_SECONDS,
                rotationSetpoint,
                rotationGoal
        );

        final double rotationFF = rotationSetpoint.velocity;
        final double rotationFeedback = rotationController.calculate(
                currentRotationRads,
                rotationSetpoint.position
        );
        final double rotationSpeed = rotationFeedback + rotationFF;

        if (Math.abs(setpointDistance - currentDistance) > AllowableDistanceMetersBeforeReset
                || Math.abs(
                        MathUtil.angleModulus(
                        setpointAngle.getRadians()
                                - currentAngle.getRadians())) > AllowableAngleRadsBeforeReset
                || Math.abs(
                        MathUtil.angleModulus(
                        rotationSetpoint.position
                                - currentRotationRads)) > AllowableHeadingRadsBeforeReset
        ) {
            reset(currentPose, targetPose);
        }

        final ChassisSpeeds fieldSpeeds = fieldRelativeSpeedsSupplier.get();
        final Translation2d linearFieldSpeeds = new Translation2d(
                fieldSpeeds.vxMetersPerSecond,
                fieldSpeeds.vyMetersPerSecond
        );
        final Pose2d setpointPose = new Pose2d(
                lastSetpointTranslation,
                Rotation2d.fromRadians(rotationSetpoint.position)
        );

        Logger.recordOutput(LogKey + "/Distance", currentDistance);
        Logger.recordOutput(LogKey + "/DistanceSetpoint", translationSetpoint.position);
        Logger.recordOutput(
                LogKey + "/Velocity",
                // Component of field speeds along direction from current to target
                -linearFieldSpeeds
                        .toVector()
                        .dot(targetToCurrent.unaryMinus().toVector())
                        / currentDistance
        );
        Logger.recordOutput(LogKey + "/VelocitySetpoint", translationSetpoint.velocity);

        Logger.recordOutput(LogKey + "/Rotation", currentRotationRads);
        Logger.recordOutput(LogKey + "/RotationSetpoint", rotationSetpoint.position);

        Logger.recordOutput(LogKey + "/RotationVelocity", fieldSpeeds.omegaRadiansPerSecond);
        Logger.recordOutput(LogKey + "/RotationVelocitySetpoint", rotationSetpoint.velocity);

        Logger.recordOutput(LogKey + "/TargetPose", targetPose);
        Logger.recordOutput(LogKey + "/Setpoint", setpointPose);

        return ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed,
                ySpeed,
                rotationSpeed,
                currentPose.getRotation()
        );
    }
}
