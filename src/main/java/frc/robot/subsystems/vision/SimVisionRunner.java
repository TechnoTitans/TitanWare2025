package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.vision.cameras.TitanCamera;
import frc.robot.subsystems.vision.estimator.VisionPoseEstimator;
import frc.robot.subsystems.vision.estimator.VisionResult;
import frc.robot.utils.closeables.ToClose;
import frc.robot.utils.gyro.GyroUtils;
import frc.robot.utils.logging.Tracer;
import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

import java.util.HashMap;
import java.util.Map;
import java.util.Optional;
import java.util.function.Function;

public class SimVisionRunner implements PhotonVisionRunner {
    public static class VisionIOApriltagsSim implements VisionIO {
        public final PhotonCamera photonCamera;
        public final String cameraName;

        public final double stdDevFactor;
        public final Transform3d robotToCamera;
        public final PhotonPoseEstimator.ConstrainedSolvepnpParams constrainedPnpParams;

        public VisionIOApriltagsSim(final TitanCamera titanCamera, final VisionSystemSim visionSystemSim) {
            this.photonCamera = titanCamera.getPhotonCamera();
            this.cameraName = photonCamera.getName();

            this.stdDevFactor = titanCamera.getStdDevFactor();
            this.robotToCamera = titanCamera.getRobotToCameraTransform();
            this.constrainedPnpParams = titanCamera.getConstrainedPnpParams();

            final PhotonCameraSim photonCameraSim =
                    new PhotonCameraSim(titanCamera.getPhotonCamera(), titanCamera.toSimCameraProperties());

            photonCameraSim.enableDrawWireframe(true);
            photonCameraSim.enableRawStream(true);
            photonCameraSim.enableProcessedStream(true);

            ToClose.add(photonCameraSim);
            visionSystemSim.addCamera(photonCameraSim, titanCamera.getRobotToCameraTransform());
        }

        @Override
        public void updateInputs(final VisionIOInputs inputs) {
            inputs.name = cameraName;
            inputs.isConnected = photonCamera.isConnected();
            inputs.stdDevFactor = stdDevFactor;
            inputs.constrainedPnpParams = constrainedPnpParams;
            inputs.robotToCamera = robotToCamera;
            inputs.cameraMatrix = photonCamera.getCameraMatrix().orElse(EmptyCameraMatrix);
            inputs.distortionCoeffs = photonCamera.getDistCoeffs().orElse(EmptyDistortionCoeffs);
            inputs.pipelineResults = photonCamera.getAllUnreadResults().toArray(new PhotonPipelineResult[0]);
        }
    }

    private final Swerve swerve;
    private final SwerveDriveOdometry visionIndependentOdometry;
    private final VisionSystemSim visionSystemSim;

    private final AprilTagFieldLayout aprilTagFieldLayout;
    private final Map<VisionIOApriltagsSim, VisionIO.VisionIOInputs> apriltagVisionIOInputsMap;

    private final Map<VisionIO, VisionResult> visionResults;

    public SimVisionRunner(
            final Swerve swerve,
            final SwerveDriveOdometry visionIndependentOdometry,
            final AprilTagFieldLayout aprilTagFieldLayout,
            final VisionSystemSim visionSystemSim,
            final Map<VisionIOApriltagsSim, VisionIO.VisionIOInputs> apriltagVisionIOInputsMap
    ) {
        this.swerve = swerve;
        this.visionIndependentOdometry = visionIndependentOdometry;
        this.visionSystemSim = visionSystemSim;
        this.visionSystemSim.addAprilTags(aprilTagFieldLayout);

        this.aprilTagFieldLayout = aprilTagFieldLayout;
        this.apriltagVisionIOInputsMap = apriltagVisionIOInputsMap;

        this.visionResults = new HashMap<>();
    }

    @SuppressWarnings("DuplicatedCode")
    @Override
    public void periodic(final Function<Double, Optional<Pose2d>> poseAtTimestamp) {
        Tracer.trace("SimVisionRunner");
        if (ToClose.hasClosed()) {
            return;
        }

        final Pose2d visionIndependentPose =
                visionIndependentOdometry.update(swerve.getYaw(), swerve.getModulePositions());

        Tracer.trace("VisionSystemSim");
        visionSystemSim.update(
                GyroUtils.robotPose2dToPose3dWithGyro(
                        visionIndependentPose,
                        new Rotation3d(
                                swerve.getRoll().getRadians(),
                                swerve.getPitch().getRadians(),
                                swerve.getYaw().getRadians()
                        )
                )
        );
        Tracer.stop();

        for (
                final Map.Entry<VisionIOApriltagsSim, VisionIO.VisionIOInputs>
                        photonVisionIOInputsEntry : apriltagVisionIOInputsMap.entrySet()
        ) {
            final VisionIOApriltagsSim visionIO = photonVisionIOInputsEntry.getKey();
            final VisionIO.VisionIOInputs inputs = photonVisionIOInputsEntry.getValue();

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
                        visionIO.robotToCamera,
                        pipelineResult,
                        inputs.cameraMatrix,
                        inputs.distortionCoeffs,
                        visionIO.constrainedPnpParams
                );

                visionResults.put(
                        visionIO,
                        visionResult
                );
            }
        }
        Tracer.stop();
    }

    /**
     * Reset the simulated robot {@link Pose3d}.
     * @param robotPose the new robot {@link Pose3d}
     */
    @Override
    public void resetRobotPose(final Pose3d robotPose) {
        final Pose2d currentPose = robotPose.toPose2d();

        visionIndependentOdometry.resetPosition(
                currentPose.getRotation(), swerve.getModulePositions(), currentPose
        );
        visionSystemSim.resetRobotPose(robotPose);
    }

    @Override
    public Map<VisionIOApriltagsSim, VisionIO.VisionIOInputs> getApriltagVisionIOInputsMap() {
        return apriltagVisionIOInputsMap;
    }

    @Override
    public VisionResult getVisionResult(final VisionIO visionIO) {
        return visionResults.get(visionIO);
    }
}
