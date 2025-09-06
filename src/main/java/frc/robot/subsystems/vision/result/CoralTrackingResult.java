package frc.robot.subsystems.vision.result;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.Constants;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.Optional;
import java.util.function.Function;

public class CoralTrackingResult {
    private static final double DistanceOffsetMeters = Units.inchesToMeters(0); //Bounding box Offset (Selected in PV)
    private static final double AllowableTimestampTimeout = 1;

    private final Transform3d robotToCamera;

    public final PhotonPipelineResult pipelineResult;
    public final boolean hasTargets;

    public final PhotonTrackedTarget bestTarget;
    public final Rotation2d bestTargetYaw;
    public final Rotation2d bestTargetPitch;
    public final double bestTargetDistance;

    public CoralTrackingResult(final Transform3d robotToCamera, final PhotonPipelineResult pipelineResult) {
        this.robotToCamera = robotToCamera;
        this.pipelineResult = pipelineResult;

        if (pipelineResult == null || !pipelineResult.hasTargets()) {
            this.hasTargets = false;
            this.bestTarget = null;
            this.bestTargetYaw = new Rotation2d();
            this.bestTargetPitch = new Rotation2d();
            this.bestTargetDistance = 0;
        } else {
            this.hasTargets = true;
            this.bestTarget = pipelineResult.getBestTarget();
            this.bestTargetYaw = getTargetYaw(bestTarget);
            this.bestTargetPitch = getTargetPitch(bestTarget);
            this.bestTargetDistance = getCoralDistance(robotToCamera, bestTarget);
        }
    }

    public Optional<Pose2d> getBestCoralPose(final Function<Double, Optional<Pose2d>> timestampedRobotPoseFunction) {
        if (!hasTargets) {
            return Optional.empty();
        }

        if (Timer.getFPGATimestamp() - pipelineResult.getTimestampSeconds() > AllowableTimestampTimeout) {
            return Optional.empty();
        }

        final Optional<Pose2d> optionalRobotPose = timestampedRobotPoseFunction.apply(pipelineResult.getTimestampSeconds());
        return optionalRobotPose.map(pose2d -> getCoralPose(pose2d, robotToCamera, bestTarget));
    }

    public Pose2d[] getCoralPoses(final Function<Double, Optional<Pose2d>> timestampedRobotPoseFunction) {
        if (!hasTargets) {
            return new Pose2d[]{};
        }

        final Optional<Pose2d> optionalRobotPose = timestampedRobotPoseFunction.apply(pipelineResult.getTimestampSeconds());
        if (optionalRobotPose.isEmpty()) {
            return new Pose2d[]{};
        }

        if (Timer.getFPGATimestamp() - pipelineResult.getTimestampSeconds() > AllowableTimestampTimeout) {
            return new Pose2d[]{};
        }

        final int nTargets = pipelineResult.targets.size();
        final Pose2d robotPose = optionalRobotPose.get();
        final Pose2d[] CoralPoses = new Pose2d[nTargets];
        for (int i = 0; i < nTargets; i++) {
            CoralPoses[i] = getCoralPose(robotPose, robotToCamera, pipelineResult.targets.get(i));
        }

        return CoralPoses;
    }

    public static Rotation2d getTargetYaw(final PhotonTrackedTarget trackedTarget) {
        return Rotation2d.fromDegrees(-trackedTarget.getYaw());
    }

    public static Rotation2d getTargetPitch(final PhotonTrackedTarget trackedTarget) {
        return Rotation2d.fromDegrees(-trackedTarget.getPitch());
    }

    public static double getCoralDistance(final Transform3d robotToCamera, final PhotonTrackedTarget trackedTarget) {
        return PhotonUtils.calculateDistanceToTargetMeters(
                robotToCamera.getZ(),
                Constants.Vision.CORAL_Z_POSITION,
                -robotToCamera.getRotation().getY(), //Convert CCW+ to CW+
                Units.degreesToRadians(trackedTarget.getPitch())
        ) / Math.cos(Units.degreesToRadians(trackedTarget.getYaw())) + DistanceOffsetMeters;
    }

    public static Pose2d getCoralPose(
            final Pose2d poseAtTimestamp,
            final Transform3d robotToCamera,
            final PhotonTrackedTarget trackedTarget
    ) {
        final Rotation2d targetYaw = getTargetYaw(trackedTarget);

        final Translation2d estimatedTargetTranslation = new Translation2d(
                getCoralDistance(robotToCamera, trackedTarget),
                targetYaw
        ).rotateBy(robotToCamera.getRotation().toRotation2d())
                .plus(robotToCamera.getTranslation().toTranslation2d());

        final Transform2d estimatedTransform = new Transform2d(
                estimatedTargetTranslation,
                targetYaw
        );

        return poseAtTimestamp.transformBy(estimatedTransform);
    }
}
