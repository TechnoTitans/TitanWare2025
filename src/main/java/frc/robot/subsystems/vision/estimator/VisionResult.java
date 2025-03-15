package frc.robot.subsystems.vision.estimator;

import edu.wpi.first.math.geometry.Pose3d;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.List;
import java.util.Optional;

public record VisionResult(
        Result result,
        Optional<VisionUpdate> visionUpdate
) {
    public enum Result {
        NEGATIVE_TIMESTAMP,
        NO_TARGETS,
        SINGLE_TARGET_RESULT,
        SINGLE_TARGET_INVALID_TAG,
        SINGLE_TARGET_AMBIGUOUS_NO_POSE,
        SINGLE_TARGET_DISAMBIGUATE_POSE0_RESULT,
        SINGLE_TARGET_DISAMBIGUATE_POSE1_RESULT,
        MULTI_TARGET_RESULT,
        CONSTRAINED_PNP_RESULT,
        CONSTRAINED_PNP_NO_SEED_POSE
    }

    public record VisionUpdate(
            Pose3d estimatedPose,
            double timestamp,
            List<PhotonTrackedTarget> targetsUsed
    ) {}

    public static VisionResult invalid(final Result result) {
        return new VisionResult(result, Optional.empty());
    }

    public static VisionResult valid(final Result result, VisionUpdate update) {
        return new VisionResult(result, Optional.of(update));
    }
}
