package frc.robot.subsystems.vision.cameras;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N8;
import org.photonvision.simulation.SimCameraProperties;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

@SuppressWarnings("UnusedReturnValue")
public class TitanCameraCalibration {
    private final Map<CameraProperties.Resolution, SimCameraProperties> cameraPropertiesMap;

    public TitanCameraCalibration(final Map<CameraProperties.Resolution, SimCameraProperties> cameraPropertiesMap) {
        this.cameraPropertiesMap = cameraPropertiesMap;
    }

    public TitanCameraCalibration() {
        this(new HashMap<>());
    }

    public static TitanCameraCalibration perfect(final CameraProperties cameraProperties) {
        final List<CameraProperties.Resolution> resolutions = cameraProperties.getResolutions();
        final Map<CameraProperties.Resolution, SimCameraProperties> cameraPropertiesMap = new HashMap<>();

        for (final CameraProperties.Resolution resolution : resolutions) {
            final SimCameraProperties simCameraProperties = new SimCameraProperties();
            simCameraProperties.setCalibration(
                    resolution.getWidth(),
                    resolution.getHeight(),
                    Rotation2d.fromDegrees(cameraProperties.getCamDiagonalFOVDeg())
            );
            cameraPropertiesMap.put(resolution, simCameraProperties);
        }

        return new TitanCameraCalibration(cameraPropertiesMap);
    }

    public static TitanCameraCalibration fromSimCameraProperties(final SimCameraProperties simCameraProperties) {
        final int width = simCameraProperties.getResWidth();
        final int height = simCameraProperties.getResHeight();

        final CameraProperties.Resolution resolution =
                CameraProperties.Resolution
                        .getFromWidthHeight(width, height)
                        .orElseThrow(() -> new RuntimeException(
                                String.format(
                                        "Failed to find resolution of %s",
                                        CameraProperties.Resolution.rawToString(width, height)
                                )
                        ));

        return new TitanCameraCalibration(new HashMap<>(Map.of(resolution, simCameraProperties)));
    }

    private static double getPxAvgError(final List<Double> pxErrors) {
        double sum = 0;
        for (final double pxError : pxErrors) {
            sum += pxError;
        }
        return sum / pxErrors.size();
    }

    private SimCameraProperties getOrMake(final CameraProperties.Resolution resolution) {
        final SimCameraProperties simCameraProperties = cameraPropertiesMap.get(resolution);
        if (simCameraProperties != null) {
            return simCameraProperties;
        }

        final SimCameraProperties newSimCameraProperties = new SimCameraProperties();
        cameraPropertiesMap.put(resolution, newSimCameraProperties);

        return newSimCameraProperties;
    }

    private TitanCameraCalibration withIntrinsics(
            final CameraProperties.Resolution resolution,
            final Matrix<N3, N3> intrinsics
    ) {
        final SimCameraProperties simCameraProperties = getOrMake(resolution);
        simCameraProperties.setCalibration(
                resolution.getWidth(), resolution.getHeight(), intrinsics, simCameraProperties.getDistCoeffs()
        );

        return this;
    }

    private TitanCameraCalibration withDistortions(
            final CameraProperties.Resolution resolution,
            final Matrix<N8, N1> distortions
    ) {
        final SimCameraProperties simCameraProperties = getOrMake(resolution);
        simCameraProperties.setCalibration(
                resolution.getWidth(), resolution.getHeight(), simCameraProperties.getIntrinsics(), distortions
        );

        return this;
    }

    public TitanCameraCalibration withCalibration(
            final CameraProperties.Resolution resolution,
            final Matrix<N3, N3> intrinsics,
            final Matrix<N8, N1> distortions
    ) {
        withIntrinsics(resolution, intrinsics);
        withDistortions(resolution, distortions);

        return this;
    }

    public TitanCameraCalibration withCalibrationError(
            final CameraProperties.Resolution resolution,
            final double avgPxError,
            final double pxErrorStdDev
    ) {
        final SimCameraProperties simCameraProperties = getOrMake(resolution);
        simCameraProperties.setCalibError(avgPxError, pxErrorStdDev);

        return this;
    }

    public TitanCameraCalibration withFPS(
            final CameraProperties.Resolution resolution,
            final double avgFPS
    ) {
        final SimCameraProperties simCameraProperties = getOrMake(resolution);
        simCameraProperties.setFPS(avgFPS);

        return this;
    }

    public TitanCameraCalibration withLatency(
            final CameraProperties.Resolution resolution,
            final double avgLatencyMs,
            final double latencyStdDevMs
    ) {
        final SimCameraProperties simCameraProperties = getOrMake(resolution);
        simCameraProperties.setAvgLatencyMs(avgLatencyMs);
        simCameraProperties.setLatencyStdDevMs(latencyStdDevMs);

        return this;
    }

    public boolean hasResolution(final CameraProperties.Resolution resolution) {
        return cameraPropertiesMap.containsKey(resolution);
    }

    public SimCameraProperties getSimCameraProperties(final CameraProperties.Resolution resolution) {
        return cameraPropertiesMap.get(resolution);
    }

    public Matrix<N3, N3> getIntrinsics(final CameraProperties.Resolution resolution) {
        return getSimCameraProperties(resolution).getIntrinsics();
    }

    public Matrix<N8, N1> getDistortions(final CameraProperties.Resolution resolution) {
        return getSimCameraProperties(resolution).getDistCoeffs();
    }
}
