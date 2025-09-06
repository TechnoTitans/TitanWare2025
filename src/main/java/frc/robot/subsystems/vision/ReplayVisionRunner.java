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

    private final Map<VisionIO, VisionResult[]> visionResultsByVisionIO;
    private final Map<VisionIO, CoralTrackingResult[]> coralTrackingResultsByVisionIO;

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
        this.visionResultsByVisionIO = new HashMap<>();
        this.coralTrackingResultsByVisionIO = new HashMap<>();
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
            final int nPipelineResults = pipelineResults.length;
            final VisionResult[] visionResults = new VisionResult[pipelineResults.length];

            for (int i = 0; i < nPipelineResults; i++) {
                final PhotonPipelineResult pipelineResult = pipelineResults[i];
                final VisionResult visionResult = VisionPoseEstimator.update(
                        aprilTagFieldLayout,
                        poseAtTimestamp,
                        inputs,
                        pipelineResult
                );

                visionResults[i] = visionResult;
            }

            visionResultsByVisionIO.put(
                    visionIO,
                    visionResults
            );
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
            final int nPipelineResults = pipelineResults.length;
            final CoralTrackingResult[] coralTrackingResults = new CoralTrackingResult[nPipelineResults];

            for (int i = 0; i < nPipelineResults; i++) {
                final PhotonPipelineResult pipelineResult = pipelineResults[i];
                final CoralTrackingResult coralTrackingResult = new CoralTrackingResult(
                        inputs.robotToCamera,
                        pipelineResult
                );

                coralTrackingResults[i] = coralTrackingResult;
            }

            coralTrackingResultsByVisionIO.put(visionIO, coralTrackingResults);
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
    public VisionResult[] getVisionResults(final VisionIO visionIO) {
        return visionResultsByVisionIO.get(visionIO);
    }

    @Override
    public CoralTrackingResult[] getCoralTrackingResults(final VisionIO visionIO) {
        return coralTrackingResultsByVisionIO.get(visionIO);
    }
}
