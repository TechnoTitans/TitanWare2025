package frc.robot.subsystems.vision.estimator;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.cscore.OpenCvLoader;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N8;
import frc.robot.constants.Constants;
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
    static {
        //noinspection ResultOfMethodCallIgnored
        OpenCvLoader.forceStaticLoad();
    }

    private VisionPoseEstimator() {}

    private static VisionResult constrainedPnpStrategy(
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
                        fieldLayout,
                        robotToCamera,
                        timestamp,
                        result,
                        maybeMultiTargetResult.get()
                );
            } else {
                return singleTag(
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
            final VisionResult maybeSingleTag =
                    singleTag(fieldLayout, robotToCamera, timestamp, poseAtTimestamp, result);
            final Optional<VisionResult.VisionUpdate> visionUpdate = maybeSingleTag.visionUpdate();
            if (visionUpdate.isEmpty()) {
                return VisionResult.invalid(VisionResult.Result.CONSTRAINED_PNP_NO_SEED_POSE);
            }

            fieldToRobotSeed = visionUpdate.get().estimatedPose();
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
            return singleTag(fieldLayout, robotToCamera, timestamp, poseAtTimestamp, result);
        }

        final Pose3d best = Pose3d.kZero.plus(pnpResult.get().best); // field-to-robot

        return VisionResult.valid(
                VisionResult.Result.CONSTRAINED_PNP_RESULT,
                new VisionResult.VisionUpdate(
                        best,
                        timestamp,
                        result.getTargets()
                )
        );
    }

    private static VisionResult multitag(
            final AprilTagFieldLayout fieldLayout,
            final Transform3d robotToCamera,
            final double poseTimestamp,
            final PhotonPipelineResult pipelineResult,
            final MultiTargetPNPResult result
    ) {
        final Transform3d best = result.estimatedPose.best;
        final Pose3d bestPose =
                Pose3d.kZero
                        .plus(best) // field-to-camera
                        .relativeTo(fieldLayout.getOrigin())
                        .plus(robotToCamera.inverse()); // field-to-robot
        return VisionResult.valid(
                VisionResult.Result.MULTI_TARGET_RESULT,
                new VisionResult.VisionUpdate(
                        bestPose,
                        poseTimestamp,
                        pipelineResult.getTargets()
                )
        );
    }

    private static VisionResult singleTag(
            final AprilTagFieldLayout fieldLayout,
            final Transform3d robotToCamera,
            final double poseTimestamp,
            final Function<Double, Optional<Pose2d>> poseAtTimestamp,
            final PhotonPipelineResult pipelineResult
    ) {
        final PhotonTrackedTarget target = pipelineResult.getTargets().get(0);

        final Optional<Pose3d> maybeTagPose = fieldLayout.getTagPose(target.getFiducialId());
        if (maybeTagPose.isEmpty()) {
            return VisionResult.invalid(VisionResult.Result.SINGLE_TARGET_INVALID_TAG);
        }

        final Pose3d tagPose = maybeTagPose.get();
        final Pose3d cameraPose0 = tagPose.transformBy(target.getBestCameraToTarget().inverse());
        final Pose3d cameraPose1 = tagPose.transformBy(target.getAlternateCameraToTarget().inverse());

        final Pose3d robotPose0 = cameraPose0.transformBy(robotToCamera.inverse());
        final Pose3d robotPose1 = cameraPose1.transformBy(robotToCamera.inverse());

        if (target.getPoseAmbiguity() < Constants.Vision.MAX_ACCEPT_BEST_POSE_AMBIGUITY) {
            return VisionResult.valid(
                    VisionResult.Result.SINGLE_TARGET_RESULT,
                    new VisionResult.VisionUpdate(
                            robotPose0,
                            poseTimestamp,
                            pipelineResult.getTargets()
                    )
            );
        } else {
            if (true) return VisionResult.invalid(VisionResult.Result.SINGLE_TARGET_AMBIGUOUS_NO_POSE);
        }

        final Optional<Pose2d> maybePose = poseAtTimestamp.apply(poseTimestamp);
        if (maybePose.isEmpty()) {
            return VisionResult.invalid(VisionResult.Result.SINGLE_TARGET_AMBIGUOUS_NO_POSE);
        }

        final Pose2d robotPose = maybePose.get();
        final Rotation2d yawDifference0 = robotPose0.getRotation().toRotation2d()
                .minus(robotPose.getRotation());
        final Rotation2d yawDifference1 = robotPose1.getRotation().toRotation2d()
                .minus(robotPose.getRotation());

        if (yawDifference0.getRadians() < yawDifference1.getRadians()) {
            return VisionResult.valid(
                    VisionResult.Result.SINGLE_TARGET_DISAMBIGUATE_POSE0_RESULT,
                    new VisionResult.VisionUpdate(
                            robotPose0,
                            poseTimestamp,
                            pipelineResult.getTargets()
                    )
            );
        } else {
            return VisionResult.valid(
                    VisionResult.Result.SINGLE_TARGET_DISAMBIGUATE_POSE1_RESULT,
                    new VisionResult.VisionUpdate(
                            robotPose1,
                            poseTimestamp,
                            pipelineResult.getTargets()
                    )
            );
        }
    }

    public static VisionResult update(
            final AprilTagFieldLayout fieldLayout,
            final Function<Double, Optional<Pose2d>> poseAtTimestamp,
            final Transform3d robotToCamera,
            final PhotonPipelineResult pipelineResult,
            final Matrix<N3, N3> cameraMatrix,
            final Matrix<N8, N1> distCoeffs,
            final PhotonPoseEstimator.ConstrainedSolvepnpParams constrainedPnpParams
    ) {
        final double resultTimestampSeconds = pipelineResult.getTimestampSeconds();

        // Time in the past -- give up, since the following if expects times > 0
        if (resultTimestampSeconds < 0) {
            return VisionResult.invalid(VisionResult.Result.NEGATIVE_TIMESTAMP);
        }

        // If no targets seen, trivial case -- return empty result
        if (!pipelineResult.hasTargets()) {
            return VisionResult.invalid(VisionResult.Result.NO_TARGETS);
        }

        return pipelineResult.multitagResult.map(multiTargetPNPResult -> multitag(
                fieldLayout,
                robotToCamera,
                resultTimestampSeconds,
                pipelineResult,
                multiTargetPNPResult
        )).orElseGet(() -> singleTag(
                fieldLayout,
                robotToCamera,
                pipelineResult.getTimestampSeconds(),
                poseAtTimestamp,
                pipelineResult
        ));

//        return constrainedPnpStrategy(
//                fieldLayout,
//                pipelineResult,
//                poseAtTimestamp,
//                robotToCamera,
//                cameraMatrix,
//                distCoeffs,
//                constrainedPnpParams
//        );
    }
}
