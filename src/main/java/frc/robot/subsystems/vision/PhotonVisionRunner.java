package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.subsystems.vision.estimator.VisionResult;
import frc.robot.subsystems.vision.result.CoralTrackingResult;

import java.util.Map;
import java.util.Optional;
import java.util.function.Function;

public interface PhotonVisionRunner {
    default void periodic(final Function<Double, Optional<Pose2d>> poseAtTimestamp) {}
    default void resetRobotPose(final Pose3d pose3d) {}

    default Map<? extends VisionIO, VisionIO.VisionIOInputs> getApriltagVisionIOInputsMap() {
        return Map.of();
    }

    default Map<? extends VisionIO, VisionIO.VisionIOInputs> getCoralTrackingVisionIOInputsMap() {
        return Map.of();
    }

    default VisionResult[] getVisionResults(final VisionIO visionIO) {
        return null;
    }

    default CoralTrackingResult[] getCoralTrackingResults(final VisionIO visionIO){
        return null;
    }
}
