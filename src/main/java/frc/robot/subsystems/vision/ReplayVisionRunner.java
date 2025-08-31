package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.subsystems.vision.cameras.TitanCamera;
import frc.robot.subsystems.vision.estimator.VisionPoseEstimator;
import frc.robot.subsystems.vision.estimator.VisionResult;
import frc.robot.subsystems.vision.result.CoralTrackingResult;
import frc.robot.utils.closeables.ToClose;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import java.util.HashMap;
import java.util.Map;
import java.util.Optional;
import java.util.function.Function;

public class ReplayVisionRunner implements PhotonVisionRunner {
    public static class VisionIOReplay implements VisionIO {
        private final PhotonCamera photonCamera;

        public VisionIOReplay(final TitanCamera titanCamera) {
            this.photonCamera = titanCamera.getPhotonCamera();
        }
    }

    final AprilTagFieldLayout aprilTagFieldLayout;

    private final Map<VisionIOReplay, String> visionIONames;
    private final Map<VisionIOReplay, VisionIO.VisionIOInputs> apriltagVisionIOInputsMap;
    private final Map<VisionIOReplay, VisionIO.VisionIOInputs> coralTrackingVisionIOInputsMap;

    private final Map<VisionIO, VisionResult> visionResults;
    private final Map<VisionIO, CoralTrackingResult> coralTrackingResults;

    public ReplayVisionRunner(
            final AprilTagFieldLayout aprilTagFieldLayout,
            final Map<VisionIOReplay, VisionIO.VisionIOInputs> apriltagVisionIOInputsMap,
            final Map<VisionIOReplay, VisionIO.VisionIOInputs> coralTrackingVisionIOInputsMap
    ) {
        this.aprilTagFieldLayout = aprilTagFieldLayout;
        this.apriltagVisionIOInputsMap = apriltagVisionIOInputsMap;
        this.coralTrackingVisionIOInputsMap = coralTrackingVisionIOInputsMap;

        final Map<VisionIOReplay, String> visionIONames = new HashMap<>();
        for (final VisionIOReplay visionIOApriltagsReplay : apriltagVisionIOInputsMap.keySet()) {
            visionIONames.put(visionIOApriltagsReplay, visionIOApriltagsReplay.photonCamera.getName());
        }

        this.visionIONames = visionIONames;
        this.visionResults = new HashMap<>();
        this.coralTrackingResults = new HashMap<>();
    }

    @SuppressWarnings("DuplicatedCode")
    @Override
    public void periodic(final Function<Double, Optional<Pose2d>> poseAtTimestamp) {
        if (ToClose.hasClosed()) {
            return;
        }

        for (
                final Map.Entry<VisionIOReplay, VisionIO.VisionIOInputs>
                        photonVisionIOInputsEntry : apriltagVisionIOInputsMap.entrySet()
        ) {
            final VisionIOReplay visionIO = photonVisionIOInputsEntry.getKey();
            final VisionIO.VisionIOInputs inputs = photonVisionIOInputsEntry.getValue();

            visionIO.periodic();
            visionIO.updateInputs(inputs);

            Logger.processInputs(
                    String.format("%s/%s", PhotonVision.PhotonLogKey, visionIONames.get(visionIO)),
                    inputs
            );

            final PhotonPipelineResult[] pipelineResults = inputs.pipelineResults;
            for (final PhotonPipelineResult pipelineResult : pipelineResults) {
                final VisionResult visionResult = VisionPoseEstimator.update(
                        aprilTagFieldLayout,
                        poseAtTimestamp,
                        inputs,
                        pipelineResult
                );

                visionResults.put(
                        visionIO,
                        visionResult
                );
            }
        }

        for (
                final Map.Entry<VisionIOReplay, VisionIO.VisionIOInputs>
                    coralTrackingVisionIOInputsEntry: coralTrackingVisionIOInputsMap.entrySet()
        ) {
            final VisionIOReplay visionIO = coralTrackingVisionIOInputsEntry.getKey();
            final VisionIO.VisionIOInputs inputs = coralTrackingVisionIOInputsEntry.getValue();

            visionIO.periodic();
            visionIO.updateInputs(inputs);

            Logger.processInputs(
                    String.format("%s/%s", PhotonVision.PhotonLogKey, visionIONames.get(visionIO)),
                    inputs
            );

            final PhotonPipelineResult[] pipelineResults = inputs.pipelineResults;
            for (final PhotonPipelineResult pipelineResult : pipelineResults) {
                final CoralTrackingResult coralTrackingResult =
                        new CoralTrackingResult(inputs.robotToCamera, pipelineResult);

                coralTrackingResults.put(
                        visionIO,
                        coralTrackingResult
                );
            }


        }
    }

    /**
     * Reset the simulated robot {@link Pose3d}.
     * @param robotPose the new robot {@link Pose3d}
     */
    @Override
    public void resetRobotPose(final Pose3d robotPose) {}

    @Override
    public Map<VisionIOReplay, VisionIO.VisionIOInputs> getApriltagVisionIOInputsMap() {
        return apriltagVisionIOInputsMap;
    }

    @Override
    public Map<? extends VisionIO, VisionIO.VisionIOInputs> getCoralTrackingVisionIOInputsMap() {
        return coralTrackingVisionIOInputsMap;
    }

    @Override
    public CoralTrackingResult getCoralTrackingResult(VisionIO visionIO) {
        return coralTrackingResults.get(visionIO);
    }

    @Override
    public VisionResult getVisionResult(final VisionIO visionIO) {
        return visionResults.get(visionIO);
    }
}
