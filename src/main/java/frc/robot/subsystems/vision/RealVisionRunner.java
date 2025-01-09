package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.subsystems.vision.cameras.TitanCamera;
import frc.robot.subsystems.vision.estimator.VisionPoseEstimator;
import frc.robot.subsystems.vision.estimator.VisionUpdate;
import frc.robot.utils.closeables.ToClose;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import java.util.HashMap;
import java.util.Map;

public class RealVisionRunner implements PhotonVisionRunner {
    public static class VisionIOApriltagReal implements VisionIO {
        private final PhotonCamera photonCamera;
        private final String cameraName;

        private final double stdDevFactor;
        private final Transform3d robotToCamera;

        public VisionIOApriltagReal(final TitanCamera titanCamera) {
            this.photonCamera = titanCamera.getPhotonCamera();
            this.cameraName = photonCamera.getName();

            this.stdDevFactor = titanCamera.getStdDevFactor();
            this.robotToCamera = titanCamera.getRobotToCameraTransform();
        }

        @Override
        public void updateInputs(final VisionIOInputs inputs) {
            inputs.name = cameraName;
            inputs.stdDevFactor = stdDevFactor;
            inputs.robotToCamera = robotToCamera;
            inputs.pipelineResults = photonCamera.getAllUnreadResults().toArray(new PhotonPipelineResult[0]);
        }
    }

    private final AprilTagFieldLayout aprilTagFieldLayout;

    private final Map<VisionIOApriltagReal, VisionIO.VisionIOInputs> apriltagVisionIOInputsMap;

    private final Map<VisionIO, VisionUpdate> visionUpdates;

    public RealVisionRunner(
            final AprilTagFieldLayout aprilTagFieldLayout,
            final Map<VisionIOApriltagReal, VisionIO.VisionIOInputs> apriltagVisionIOInputsMap
    ) {
        this.aprilTagFieldLayout = aprilTagFieldLayout;

        this.apriltagVisionIOInputsMap = apriltagVisionIOInputsMap;

        this.visionUpdates = new HashMap<>();
    }

    @SuppressWarnings("DuplicatedCode")
    @Override
    public void periodic(final Pose2d currentRobotPose) {
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
            for (final PhotonPipelineResult result : pipelineResults) {
                VisionPoseEstimator.update(
                        inputs.name,
                        aprilTagFieldLayout,
                        currentRobotPose,
                        visionIO.robotToCamera,
                        result
                ).ifPresent(
                        visionUpdate -> visionUpdates.put(visionIO, visionUpdate)
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
    public Map<VisionIOApriltagReal, VisionIO.VisionIOInputs> getApriltagVisionIOInputsMap() {
        return apriltagVisionIOInputsMap;
    }

    @Override
    public VisionUpdate getVisionUpdate(final VisionIO visionIO) {
        return visionUpdates.get(visionIO);
    }
}
