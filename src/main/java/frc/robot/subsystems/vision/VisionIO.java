package frc.robot.subsystems.vision;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N8;
import frc.robot.constants.Constants;
import frc.robot.utils.logging.LogUtils;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

public interface VisionIO {
    PhotonPoseEstimator.ConstrainedSolvepnpParams DefaultConstrainedPnpParams =
            new PhotonPoseEstimator.ConstrainedSolvepnpParams(false, 1);
    Matrix<N3, N3> EmptyCameraMatrix = MatBuilder.fill(Nat.N3(), Nat.N3(), 0, 0, 0, 0, 0, 0, 0, 0, 0);
    Matrix<N8, N1> EmptyDistortionCoeffs = MatBuilder.fill(Nat.N8(), Nat.N1(), 0, 0, 0, 0, 0, 0, 0, 0);

    class VisionIOInputs implements LoggableInputs {
        public String name = "";
        public boolean isConnected = false;
        public double stdDevFactor = 1.0;
        public Transform3d robotToCamera;
        public PhotonPoseEstimator.ConstrainedSolvepnpParams constrainedPnpParams;
        public PhotonPipelineResult latestResult;

        public int resolutionWidthPx = 0;
        public int resolutionHeightPx = 0;

        public Matrix<N3, N3> cameraMatrix;
        public Matrix<N8, N1> distortionCoeffs;

        public PhotonPipelineResult[] pipelineResults;

        @Override
        public void toLog(final LogTable table) {
            table.put("Name", name);
            table.put("IsConnected", isConnected);
            table.put("StdDevFactor", stdDevFactor);
            table.put("RobotToCamera", robotToCamera);
            table.put("ConstrainedPnpParams", constrainedPnpParams);

            table.put("ResolutionWidthPx", resolutionWidthPx);
            table.put("ResolutionHeightPx", resolutionHeightPx);

            table.put("CameraMatrix", cameraMatrix.getData());
            table.put("DistortionCoeffs", distortionCoeffs.getData());

            LogUtils.serializePhotonPipelineResults(table, "LatestResult", pipelineResults);
        }

        @Override
        public void fromLog(final LogTable table) {
            this.name = table.get("Name", "unknown");
            this.isConnected = table.get("IsConnected", false);
            this.stdDevFactor = table.get("StdDevFactor", Constants.Vision.VISION_CAMERA_DEFAULT_STD_DEV_FACTOR);
            this.robotToCamera = table.get("RobotToCamera", Transform3d.kZero);
            this.constrainedPnpParams = table.get("ConstrainedPnpParams", DefaultConstrainedPnpParams);

            this.resolutionWidthPx = table.get("ResolutionWidthPx", 0);
            this.resolutionHeightPx = table.get("ResolutionHeightPx", 0);

            final double[] cameraMatrix = table.get("CameraMatrix", (double[]) null);
            this.cameraMatrix = cameraMatrix == null
                    ? EmptyCameraMatrix
                    : MatBuilder.fill(Nat.N3(), Nat.N3(), cameraMatrix);

            final double[] distortionCoeffs = table.get("DistortionCoeffs", (double[]) null);
            this.distortionCoeffs = distortionCoeffs == null
                    ? EmptyDistortionCoeffs
                    : MatBuilder.fill(Nat.N8(), Nat.N1(), distortionCoeffs);

            this.pipelineResults = LogUtils.deserializePhotonPipelineResults(table, "LatestResult");
        }
    }

    /**
     * Updates the set of loggable inputs.
     * @param inputs Logged class of IOInputs
     * @see VisionIOInputs
     */
    default void updateInputs(final VisionIOInputs inputs) {}

    default void periodic() {}
}
