package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N8;
import frc.robot.subsystems.vision.cameras.CameraProperties;
import frc.robot.subsystems.vision.cameras.TitanCamera;
import frc.robot.subsystems.vision.estimator.VisionPoseEstimator;
import frc.robot.subsystems.vision.estimator.VisionResult;
import frc.robot.subsystems.vision.result.CoralTrackingResult;
import frc.robot.utils.closeables.ToClose;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

import java.util.HashMap;
import java.util.Map;
import java.util.Optional;
import java.util.function.Function;

public class RealVisionRunner implements PhotonVisionRunner {
    public static class VisionIOApriltagReal implements VisionIO {
        private final PhotonCamera photonCamera;
        private final String cameraName;

        private final double stdDevFactor;
        private final Transform3d robotToCamera;
        private final PhotonPoseEstimator.ConstrainedSolvepnpParams constrainedPnpParams;

        private final int resolutionWidthPx;
        private final int resolutionHeightPx;

        private Matrix<N3, N3> cameraMatrix;
        private Matrix<N8, N1> distortionCoeffs;

        public VisionIOApriltagReal(final TitanCamera titanCamera) {
            this.photonCamera = titanCamera.getPhotonCamera();
            this.cameraName = photonCamera.getName();

            this.stdDevFactor = titanCamera.getStdDevFactor();
            this.robotToCamera = titanCamera.getRobotToCameraTransform();
            this.constrainedPnpParams = titanCamera.getConstrainedPnpParams();

            final CameraProperties.Resolution resolution = titanCamera
                    .getCameraProperties()
                    .getFirstResolution();
            this.resolutionWidthPx = resolution.getWidth();
            this.resolutionHeightPx = resolution.getHeight();

            this.cameraMatrix = photonCamera.getCameraMatrix().orElse(null);
            this.distortionCoeffs = photonCamera.getDistCoeffs().orElse(null);
        }

        @Override
        public void updateInputs(final VisionIOInputs inputs) {
            inputs.name = cameraName;
            inputs.isConnected = photonCamera.isConnected();
            inputs.stdDevFactor = stdDevFactor;
            inputs.robotToCamera = robotToCamera;
            inputs.constrainedPnpParams = constrainedPnpParams;

            inputs.resolutionWidthPx = resolutionWidthPx;
            inputs.resolutionHeightPx = resolutionHeightPx;

            if (cameraMatrix != null) {
                inputs.cameraMatrix = cameraMatrix;
            } else {
                final Optional<Matrix<N3, N3>> maybeCameraMatrix = photonCamera.getCameraMatrix();
                // cache cameraMatrix if possible to avoid allocation
                if (maybeCameraMatrix.isPresent()) {
                    cameraMatrix = maybeCameraMatrix.get();
                    inputs.cameraMatrix = cameraMatrix;
                } else {
                    inputs.cameraMatrix = EmptyCameraMatrix;
                }
            }

            if (distortionCoeffs != null) {
                inputs.distortionCoeffs = distortionCoeffs;
            } else {
                final Optional<Matrix<N8, N1>> maybeDistortionCoeffs = photonCamera.getDistCoeffs();
                // cache distortionCoeffs if possible to avoid allocation
                if (maybeDistortionCoeffs.isPresent()) {
                    distortionCoeffs = maybeDistortionCoeffs.get();
                    inputs.distortionCoeffs = distortionCoeffs;
                } else {
                    inputs.distortionCoeffs = EmptyDistortionCoeffs;
                }
            }

            inputs.pipelineResults = photonCamera.getAllUnreadResults().toArray(new PhotonPipelineResult[0]);
        }
    }

    public static class VisionIOCoralTrackingReal implements VisionIO {
        private final PhotonCamera photonCamera;
        private final String cameraName;

        private final Transform3d robotToCamera;

        public VisionIOCoralTrackingReal(final TitanCamera titanCamera) {
            this.photonCamera = titanCamera.getPhotonCamera();
            this.cameraName = photonCamera.getName();
            this.robotToCamera = titanCamera.getRobotToCameraTransform();
        }

        @Override
        public void updateInputs(VisionIOInputs inputs) {
            inputs.name = cameraName;
            inputs.stdDevFactor = -1;
            inputs.robotToCamera = robotToCamera;
            inputs.pipelineResults = photonCamera.getAllUnreadResults().toArray(new PhotonPipelineResult[0]);
        }
    }

    private final AprilTagFieldLayout aprilTagFieldLayout;
    private final Map<VisionIOApriltagReal, VisionIO.VisionIOInputs> apriltagVisionIOInputsMap;
    private final Map<VisionIOCoralTrackingReal, VisionIO.VisionIOInputs> coralTrackingVisionIOInputsMap;

    private final Map<VisionIO, VisionResult> visionResults;
    private final Map<VisionIO, CoralTrackingResult> coralTrackingResultMap;

    public RealVisionRunner(
            final AprilTagFieldLayout aprilTagFieldLayout,
            final Map<VisionIOApriltagReal, VisionIO.VisionIOInputs> apriltagVisionIOInputsMap,
            final Map<VisionIOCoralTrackingReal, VisionIO.VisionIOInputs> coralTrackingVisionIOInputsMap
    ) {
        this.aprilTagFieldLayout = aprilTagFieldLayout;
        this.apriltagVisionIOInputsMap = apriltagVisionIOInputsMap;
        this.coralTrackingVisionIOInputsMap = coralTrackingVisionIOInputsMap;
        this.visionResults = new HashMap<>();
        this.coralTrackingResultMap = new HashMap<>();
    }

    @SuppressWarnings("DuplicatedCode")
    @Override
    public void periodic(final Function<Double, Optional<Pose2d>> poseAtTimestamp) {
        if (ToClose.hasClosed()) {
            return;
        }

        for (
                final Map.Entry<VisionIOApriltagReal, VisionIO.VisionIOInputs>
                        visionIOInputsEntry : apriltagVisionIOInputsMap.entrySet()
        ) {
            final VisionIOApriltagReal visionIO = visionIOInputsEntry.getKey();
            final VisionIO.VisionIOInputs inputs = visionIOInputsEntry.getValue();

            visionIO.periodic();
            visionIO.updateInputs(inputs);

            Logger.processInputs(
                    String.format("%s/%s", PhotonVision.PhotonLogKey, inputs.name),
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
                final Map.Entry<VisionIOCoralTrackingReal, VisionIO.VisionIOInputs>
                    visionIOInputsEntry : coralTrackingVisionIOInputsMap.entrySet()
        ) {
            final VisionIOCoralTrackingReal visionIO = visionIOInputsEntry.getKey();
            final VisionIO.VisionIOInputs inputs = visionIOInputsEntry.getValue();

            visionIO.periodic();
            visionIO.updateInputs(inputs);

            Logger.processInputs(
                    String.format("%s/%s", PhotonVision.PhotonLogKey, inputs.name),
                    inputs
            );

            final PhotonPipelineResult[] pipelineResults = inputs.pipelineResults;
            for (final PhotonPipelineResult pipelineResult : pipelineResults) {
                final CoralTrackingResult coralTrackingResult = new CoralTrackingResult(
                        inputs.robotToCamera, pipelineResult
                );

                coralTrackingResultMap.put(visionIO, coralTrackingResult);
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
    public Map<VisionIOApriltagReal, VisionIO.VisionIOInputs> getApriltagVisionIOInputsMap() {
        return apriltagVisionIOInputsMap;
    }

    @Override
    public Map<VisionIOCoralTrackingReal, VisionIO.VisionIOInputs> getCoralTrackingVisionIOInputsMap() {
        return coralTrackingVisionIOInputsMap;
    }

    @Override
    public CoralTrackingResult getCoralTrackingResult(VisionIO visionIO) {
        return coralTrackingResultMap.get(visionIO);
    }

    @Override
    public VisionResult getVisionResult(final VisionIO visionIO) {
        return visionResults.get(visionIO);
    }
}
