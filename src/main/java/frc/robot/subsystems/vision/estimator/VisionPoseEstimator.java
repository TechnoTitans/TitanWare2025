package frc.robot.subsystems.vision.estimator;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N8;
import frc.robot.constants.Constants;
import frc.robot.subsystems.vision.PhotonVision;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.estimation.TargetModel;
import org.photonvision.estimation.VisionEstimation;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.PnpResult;

import java.util.Optional;
import java.util.function.Function;

public class VisionPoseEstimator {
    private VisionPoseEstimator() {}

    private static Optional<VisionUpdate> constrainedPnpStrategy(
            final String name,
            final AprilTagFieldLayout fieldLayout,
            final PhotonPipelineResult result,
            final Function<Double, Optional<Pose2d>> poseAtTimestamp,
            final Transform3d robotToCamera,
            final Matrix<N3, N3> cameraMatrix,
            final Matrix<N8, N1> distCoeffs,
            final PhotonPoseEstimator.ConstrainedSolvepnpParams constrainedPnpParams
    ) {
        final double timestamp = result.getTimestampSeconds();
        final Optional<Pose2d> maybeRobotPose = poseAtTimestamp.apply(timestamp);
        if (!constrainedPnpParams.headingFree() && maybeRobotPose.isEmpty()) {
            final Optional<MultiTargetPNPResult> maybeMultiTargetResult = result.multitagResult;
            //noinspection OptionalIsPresent
            if (maybeMultiTargetResult.isPresent()) {
                return multitag(
                        name,
                        fieldLayout,
                        robotToCamera,
                        timestamp,
                        result,
                        maybeMultiTargetResult.get()
                );
            } else {
                return singleTag(
                        name,
                        fieldLayout,
                        robotToCamera,
                        timestamp,
                        poseAtTimestamp,
                        result
                );
            }
        }

        // Attempt to use multi-tag to get a pose estimate seed
        Pose3d fieldToRobotSeed;
        if (result.getMultiTagResult().isPresent()) {
            fieldToRobotSeed =
                    Pose3d.kZero.plus(
                            result.getMultiTagResult().get().estimatedPose.best.plus(robotToCamera.inverse())
                    );
        } else {
            // HACK - use fallback strategy to gimme a seed pose
            // TODO - make sure nested update doesn't break state
            final Optional<VisionUpdate> maybeSingleTag =
                    singleTag(name, fieldLayout, robotToCamera, timestamp, poseAtTimestamp, result);
            if (maybeSingleTag.isEmpty()) {
                return Optional.empty();
            }

            fieldToRobotSeed = maybeSingleTag.get().estimatedPose();
        }

        final Rotation2d heading = maybeRobotPose.orElseThrow().getRotation();
        if (!constrainedPnpParams.headingFree()) {
            // If heading fixed, force rotation component
            fieldToRobotSeed =
                    new Pose3d(
                            fieldToRobotSeed.getTranslation(),
                            new Rotation3d(heading)
                    );
        }

        final Optional<PnpResult> pnpResult =
                VisionEstimation.estimateRobotPoseConstrainedSolvepnp(
                        cameraMatrix,
                        distCoeffs,
                        result.getTargets(),
                        robotToCamera,
                        fieldToRobotSeed,
                        fieldLayout,
                        TargetModel.kAprilTag36h11,
                        constrainedPnpParams.headingFree(),
                        heading,
                        constrainedPnpParams.headingScaleFactor()
                );

        // try fallback strategy if solvePNP fails for some reason
        if (pnpResult.isEmpty()) {
            return singleTag(name, fieldLayout, robotToCamera, timestamp, poseAtTimestamp, result);
        }

        final Pose3d best = Pose3d.kZero.plus(pnpResult.get().best); // field-to-robot

        return Optional.of(
                new VisionUpdate(
                        best,
                        timestamp,
                        result.getTargets()
                )
        );
    }

    private static Optional<VisionUpdate> multitag(
            final String name,
            final AprilTagFieldLayout fieldLayout,
            final Transform3d robotToCamera,
            final double poseTimestamp,
            final PhotonPipelineResult pipelineResult,
            final MultiTargetPNPResult result
    ) {
        Logger.recordOutput(
                PhotonVision.PhotonLogKey + "/" + name + "/EstimatorResult",
                "Multi-target result"
        );
        final Transform3d best = result.estimatedPose.best;
        final Pose3d bestPose =
                new Pose3d()
                        .plus(best) // field-to-camera
                        .relativeTo(fieldLayout.getOrigin())
                        .plus(robotToCamera.inverse()); // field-to-robot
        return Optional.of(
                new VisionUpdate(
                        bestPose,
                        poseTimestamp,
                        pipelineResult.getTargets()
                )
        );
    }

    private static Optional<VisionUpdate> singleTag(
            final String name,
            final AprilTagFieldLayout fieldLayout,
            final Transform3d robotToCamera,
            final double poseTimestamp,
            final Function<Double, Optional<Pose2d>> poseAtTimestamp,
            final PhotonPipelineResult pipelineResult
    ) {
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
                            poseTimestamp,
                            pipelineResult.getTargets()
                    )
            );
        }

        final Optional<Pose2d> maybePose = poseAtTimestamp.apply(poseTimestamp);
        if (maybePose.isEmpty()) {
            return Optional.empty();
        }

        final Pose2d robotPose = maybePose.get();

        final Rotation2d yawDifference0 = robotPose0.getRotation().toRotation2d()
                .minus(robotPose.getRotation());
        final Rotation2d yawDifference1 = robotPose1.getRotation().toRotation2d()
                .minus(robotPose.getRotation());

        if (yawDifference0.getRadians() < yawDifference1.getRadians()) {
            return Optional.of(
                    new VisionUpdate(
                            robotPose0,
                            poseTimestamp,
                            pipelineResult.getTargets()
                    )
            );
        } else {
            return Optional.of(
                    new VisionUpdate(
                            robotPose1,
                            poseTimestamp,
                            pipelineResult.getTargets()
                    )
            );
        }
    }

    public static Optional<VisionUpdate> update(
            final String name,
            final AprilTagFieldLayout fieldLayout,
            final Function<Double, Optional<Pose2d>> poseAtTimestamp,
            final Transform3d robotToCamera,
            final PhotonPipelineResult pipelineResult,
            final Matrix<N3, N3> cameraMatrix,
            final Matrix<N8, N1> distCoeffs,
            final PhotonPoseEstimator.ConstrainedSolvepnpParams constrainedPnpParams
    ) {
        final double poseTimestamp = pipelineResult.getTimestampSeconds();

        // Time in the past -- give up, since the following if expects times > 0
        if (poseTimestamp < 0) {
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

        return constrainedPnpStrategy(
                name,
                fieldLayout,
                pipelineResult,
                poseAtTimestamp,
                robotToCamera,
                cameraMatrix,
                distCoeffs,
                constrainedPnpParams
        );
    }
}
