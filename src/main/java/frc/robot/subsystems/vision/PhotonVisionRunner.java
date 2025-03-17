package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.subsystems.vision.estimator.VisionResult;

import java.util.Map;
import java.util.Optional;
import java.util.function.Function;

public interface PhotonVisionRunner {
    default void periodic(final Function<Double, Optional<Pose2d>> poseAtTimestamp) {}
    default void resetRobotPose(final Pose3d pose3d) {}

    default Map<? extends VisionIO, VisionIO.VisionIOInputs> getApriltagVisionIOInputsMap() {
        return Map.of();
    }

    default VisionResult getVisionResult(final VisionIO visionIO) {
        return null;
    }
}
