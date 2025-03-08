package frc.robot.subsystems.vision.estimator;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.constants.Constants;
import frc.robot.subsystems.vision.PhotonVision;
import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.Optional;

public class VisionPoseEstimator {
    private VisionPoseEstimator() {}

    public static Optional<VisionUpdate> update(
            final String name,
            final AprilTagFieldLayout fieldLayout,
            final Pose2d currentRobotPose,
            final Transform3d robotToCamera,
            final PhotonPipelineResult pipelineResult
    ) {
        // Time in the past -- give up, since the following if expects times > 0
        if (pipelineResult.getTimestampSeconds() < 0) {
            Logger.recordOutput(
                    PhotonVision.PhotonLogKey + "/" + name + "/EstimatorResult",
                    "Timestamp is negative"
            );
            return Optional.empty();
        }

        // If no targets seen, trivial case -- return empty result
        if (!pipelineResult.hasTargets()) {
            Logger.recordOutput(
                    PhotonVision.PhotonLogKey + "/" + name + "/EstimatorResult",
                    "No targets seen"
            );
            return Optional.empty();
        }

        final Optional<MultiTargetPNPResult> maybeMultiTargetResult = pipelineResult.multitagResult;
        if (maybeMultiTargetResult.isPresent()) {
            Logger.recordOutput(
                    PhotonVision.PhotonLogKey + "/" + name + "/EstimatorResult",
                    "Multi-target result"
            );
            final MultiTargetPNPResult result = maybeMultiTargetResult.get();
            final Transform3d best = result.estimatedPose.best;
            final Pose3d bestPose =
                    new Pose3d()
                            .plus(best) // field-to-camera
                            .relativeTo(fieldLayout.getOrigin())
                            .plus(robotToCamera.inverse()); // field-to-robot
            return Optional.of(
                    new VisionUpdate(
                            bestPose,
                            pipelineResult.getTimestampSeconds(),
                            pipelineResult.getTargets()
                    )
            );
        } else {
            final PhotonTrackedTarget target = pipelineResult.getTargets().get(0);

            final Optional<Pose3d> maybeTagPose = fieldLayout.getTagPose(target.getFiducialId());
            if (maybeTagPose.isEmpty()) {
                Logger.recordOutput(
                        PhotonVision.PhotonLogKey + "/" + name + "/EstimatorResult",
                        "Single-target Empty"
                );
                return Optional.empty();
            }

            Logger.recordOutput(
                    PhotonVision.PhotonLogKey + "/" + name + "/EstimatorResult",
                    "Single-target result"
            );

            final Pose3d tagPose = maybeTagPose.get();
            final Pose3d cameraPose0 = tagPose.transformBy(target.getBestCameraToTarget().inverse());
            final Pose3d cameraPose1 = tagPose.transformBy(target.getAlternateCameraToTarget().inverse());

            final Pose3d robotPose0 = cameraPose0.transformBy(robotToCamera.inverse());
            final Pose3d robotPose1 = cameraPose1.transformBy(robotToCamera.inverse());

            if (target.getPoseAmbiguity() < Constants.Vision.MAX_ACCEPT_BEST_POSE_AMBIGUITY) {
                return Optional.of(
                        new VisionUpdate(
                                robotPose0,
                                pipelineResult.getTimestampSeconds(),
                                pipelineResult.getTargets()
                        )
                );
            }

            final Rotation2d yawDifference0 = robotPose0.getRotation().toRotation2d()
                    .minus(currentRobotPose.getRotation());
            final Rotation2d yawDifference1 = robotPose1.getRotation().toRotation2d()
                    .minus(currentRobotPose.getRotation());

            if (yawDifference0.getRadians() < yawDifference1.getRadians()) {
                return Optional.of(
                        new VisionUpdate(
                                robotPose0,
                                pipelineResult.getTimestampSeconds(),
                                pipelineResult.getTargets()
                        )
                );
            } else {
                return Optional.of(
                        new VisionUpdate(
                                robotPose1,
                                pipelineResult.getTimestampSeconds(),
                                pipelineResult.getTargets()
                        )
                );
            }
        }
    }
}
