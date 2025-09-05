package frc.robot.subsystems.vision.cameras;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.constants.Constants;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.SimCameraProperties;

public enum TitanCamera {
    PHOTON_FR_APRILTAG(
            "FR_Apriltag",
            Constants.Vision.ROBOT_TO_FR_APRILTAG,
            new PhotonPoseEstimator.ConstrainedSolvepnpParams(false, 1),
            CameraProperties.SEE3CAM_24CUG,
            1,
            true,
            new TitanCameraCalibration()
                    .withCalibration(
                            CameraProperties.Resolution.R1280x720,
                            MatBuilder.fill(
                                    Nat.N3(),
                                    Nat.N3(),
                                    // intrinsic
                                    691.1847272,
                                    0.0,
                                    669.781416,
                                    0.0,
                                    691.0630782,
                                    373.655306,
                                    0.0,
                                    0.0,
                                    1.0
                            ),
                            VecBuilder.fill(
                                    // distort
                                    0.170009144,
                                    -0.09435815026,
                                    8.282499574E-6,
                                    1.31427394E-4,
                                    -0.002606170512,
                                    0.5344905386,
                                    -0.1168168982,
                                    -0.0197253292
                            )
                    )
                    .withCalibrationError(
                            CameraProperties.Resolution.R1280x720,
                            0.15223032073535464,
                            0.06
                    )
                    .withFPS(
                            CameraProperties.Resolution.R1280x720,
                            60
                    )
                    .withLatency(
                            CameraProperties.Resolution.R1280x720,
                            7,
                            20
                    ),
            false
    ),
    PHOTON_BL_APRILTAG(
            "BL_Apriltag",
            Constants.Vision.ROBOT_TO_BL_APRILTAG,
            new PhotonPoseEstimator.ConstrainedSolvepnpParams(false, 1),
            CameraProperties.SEE3CAM_24CUG,
            2,
            true,
            new TitanCameraCalibration()
                    .withCalibration(
                            CameraProperties.Resolution.R1280x720,
                            MatBuilder.fill(
                                    Nat.N3(),
                                    Nat.N3(),
                                    // intrinsic
                                    686.0622656,
                                    0.0,
                                    683.547432,
                                    0.0,
                                    686.060165,
                                    341.3336965,
                                    0.0,
                                    0.0,
                                    1.0
                            ),
                            VecBuilder.fill(
                                    // distortion
                                    0.1597115205,
                                    -0.0912664895,
                                    -1.875352567E-4,
                                    1.617147315E-4,
                                    -0.002312794241,
                                    0.5251817734,
                                    -0.1170233158,
                                    -0.01828650622
                            )
                    )
                    .withCalibrationError(
                            CameraProperties.Resolution.R1280x720,
                            0.15223032073535464,
                            0.06
                    )
                    .withFPS(
                            CameraProperties.Resolution.R1280x720,
                            60
                    )
                    .withLatency(
                            CameraProperties.Resolution.R1280x720,
                            15,
                            3
                    ),
            false
    ),
    PHOTON_FL_BOTTOM_APRILTAG(
            "FL_BOTTOM_Apriltag",
            Constants.Vision.ROBOT_TO_FL_APRILTAG,
            new PhotonPoseEstimator.ConstrainedSolvepnpParams(false, 1),
            CameraProperties.ARDUCAM_OV9281,
            2,
            true,
            new TitanCameraCalibration()
                    .withCalibration(
                            CameraProperties.Resolution.R1280x720,
                            MatBuilder.fill(
                                    Nat.N3(),
                                    Nat.N3(),
                                    // intrinsic
                                    913.8057298291733,
                                    0.0,
                                    639.4170623483981,
                                    0.0,
                                    913.3620266692185,
                                    354.2383130253713,
                                    0.0,
                                    0.0,
                                    1.0
                            ),
                            VecBuilder.fill(
                                    // distortion
                                    0.047,
                                    -0.069,
                                    0.001,
                                    0.001,
                                    0.015,
                                    -0.002,
                                    0.006,
                                    0.001
                            )
                    )
                    .withCalibrationError(
                            CameraProperties.Resolution.R1280x720,
                            0.39,
                            0.06
                    )
                    .withFPS(
                            CameraProperties.Resolution.R1280x720,
                            60
                    )
                    .withLatency(
                            CameraProperties.Resolution.R1280x720,
                            25,
                            3
                    ),
            false
    ),
    PHOTON_FC_CORAL_TRACKING(
            "FC_CORAL",
            Constants.Vision.ROBOT_TO_FC_CORAL_TRACKING,
            new PhotonPoseEstimator.ConstrainedSolvepnpParams(false, 1),
            CameraProperties.ARDUCAM_OV9281,
            2,
            true,
            new TitanCameraCalibration()
                    .withCalibration(
                            CameraProperties.Resolution.R1280x720,
                            MatBuilder.fill(
                                    Nat.N3(),
                                    Nat.N3(),
                                    // intrinsic
                                    913.8057298291733,
                                    0.0,
                                    639.4170623483981,
                                    0.0,
                                    913.3620266692185,
                                    354.2383130253713,
                                    0.0,
                                    0.0,
                                    1.0
                            ),
                            VecBuilder.fill(
                                    // distortion
                                    0.047,
                                    -0.069,
                                    0.001,
                                    0.001,
                                    0.015,
                                    -0.002,
                                    0.006,
                                    0.001
                            )
                    )
                    .withCalibrationError(
                            CameraProperties.Resolution.R1280x720,
                            0.39,
                            0.06
                    )
                    .withFPS(
                            CameraProperties.Resolution.R1280x720,
                            60
                    )
                    .withLatency(
                            CameraProperties.Resolution.R1280x720,
                            25,
                            3
                    ),
            false
    )
    ;

    private final PhotonCamera photonCamera;
    private final Transform3d robotToCameraTransform;
    private final PhotonPoseEstimator.ConstrainedSolvepnpParams constrainedPnpParams;
    private final CameraProperties cameraProperties;
    private final double stdDevFactor;
    private final TitanCameraCalibration cameraCalibration;
    private final boolean driverCam;

    TitanCamera(
            final String photonCameraName,
            final Transform3d robotToCameraTransform,
            final PhotonPoseEstimator.ConstrainedSolvepnpParams constrainedPnpParams,
            final CameraProperties cameraProperties,
            final double stdDevFactor,
            final boolean requiresCalibration,
            final TitanCameraCalibration titanCameraCalibration,
            final boolean driverCam
    ) {
        this.photonCamera = new PhotonCamera(photonCameraName);
        this.robotToCameraTransform = robotToCameraTransform;
        this.constrainedPnpParams = constrainedPnpParams;
        this.cameraProperties = cameraProperties;
        this.stdDevFactor = stdDevFactor;
        this.cameraCalibration = titanCameraCalibration;
        this.driverCam = driverCam;

        // if it isn't a driverCam, then it should have proper calibration data
        if (!driverCam && requiresCalibration) {
            for (final CameraProperties.Resolution resolution : cameraProperties.getResolutions()) {
                if (!cameraCalibration.hasResolution(resolution)) {
                    throw new RuntimeException(
                            String.format(
                                    "Camera %s(%s) does not have calibration data for specified %s",
                                    photonCameraName, cameraProperties, resolution
                            )
                    );
                }
            }
        }

        this.photonCamera.setDriverMode(driverCam);
    }

    TitanCamera(
            final String photonCameraName,
            final Transform3d robotToCameraTransform,
            final PhotonPoseEstimator.ConstrainedSolvepnpParams constrainedPnpParams,
            final CameraProperties cameraProperties,
            final boolean driverCam
    ) {
        this(
                photonCameraName,
                robotToCameraTransform,
                constrainedPnpParams,
                cameraProperties,
                1.0,
                false,
                TitanCameraCalibration.perfect(cameraProperties),
                driverCam
        );
    }

    public PhotonCamera getPhotonCamera() {
        return photonCamera;
    }

    public Transform3d getRobotToCameraTransform() {
        return robotToCameraTransform;
    }

    public PhotonPoseEstimator.ConstrainedSolvepnpParams getConstrainedPnpParams() {
        return constrainedPnpParams;
    }

    public CameraProperties getCameraProperties() {
        return cameraProperties;
    }

    public double getStdDevFactor() {
        return stdDevFactor;
    }

    public TitanCameraCalibration getCameraCalibration() {
        return cameraCalibration;
    }

    public boolean isDriverCam() {
        return driverCam;
    }

    public SimCameraProperties toSimCameraProperties(final CameraProperties.Resolution resolution) {
        return cameraCalibration.getSimCameraProperties(resolution);
    }

    public SimCameraProperties toSimCameraProperties() {
        return toSimCameraProperties(cameraProperties.getFirstResolution());
    }
}