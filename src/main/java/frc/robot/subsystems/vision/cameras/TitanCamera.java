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
                            CameraProperties.Resolution.R1920x1080,
                            MatBuilder.fill(
                                    Nat.N3(),
                                    Nat.N3(),
                                    // intrinsic
                                    1040.7268360455328,
                                    0.0,
                                    988.2378226542257,
                                    0.0,
                                    1040.4068711002674,
                                    544.230023439286,
                                    0.0,
                                    0.0,
                                    1.0
                            ),
                            VecBuilder.fill(
                                    // distort
                                    -0.35148800442491696,
                                    0.16199158054202314,
                                    0.0003847806133909519,
                                    0.000042723769639477994,
                                    -0.042523738490321664,
                                    0,
                                    0,
                                    0
                            )
                    )
                    .withCalibrationError(
                            CameraProperties.Resolution.R1920x1080,
                            0.15223032073535464,
                            0.06
                    )
                    .withFPS(
                            CameraProperties.Resolution.R1920x1080,
                            60
                    )
                    .withLatency(
                            CameraProperties.Resolution.R1920x1080,
                            7,
                            3
                    ),
            false
    ),
    PHOTON_BL_APRILTAG(
            "BL_Apriltag",
            Constants.Vision.ROBOT_TO_BL_APRILTAG,
            new PhotonPoseEstimator.ConstrainedSolvepnpParams(false, 1),
            CameraProperties.SEE3CAM_24CUG,
            1,
            true,
            new TitanCameraCalibration()
                    .withCalibration(
                            CameraProperties.Resolution.R1920x1080,
                            MatBuilder.fill(
                                    Nat.N3(),
                                    Nat.N3(),
                                    // intrinsic
                                    1040.7268360455328,
                                    0.0,
                                    988.2378226542257,
                                    0.0,
                                    1040.4068711002674,
                                    544.230023439286,
                                    0.0,
                                    0.0,
                                    1.0
                            ),
                            VecBuilder.fill(
                                    // distortion
                                    -0.35148800442491696,
                                    0.16199158054202314,
                                    0.0003847806133909519,
                                    0.000042723769639477994,
                                    -0.042523738490321664,
                                    0,
                                    0,
                                    0
                            )
                    )
                    .withCalibrationError(
                            CameraProperties.Resolution.R1920x1080,
                            0.15223032073535464,
                            0.06
                    )
                    .withFPS(
                            CameraProperties.Resolution.R1920x1080,
                            60
                    )
                    .withLatency(
                            CameraProperties.Resolution.R1920x1080,
                            7,
                            3
                    ),
            false
    ),
    PHOTON_FL_BOTTOM_APRILTAG(
            "FL_BOTTOM_Apriltag",
            Constants.Vision.ROBOT_TO_FL_BOTTOM_APRILTAG,
            new PhotonPoseEstimator.ConstrainedSolvepnpParams(false, 1),
            CameraProperties.ARDUCAM_OV9281,
            1,
            true,
            new TitanCameraCalibration()
                    .withCalibration(
                            CameraProperties.Resolution.R640x480,
                            // TODO: calibrations from real
                            SimCameraProperties.PERFECT_90DEG().getIntrinsics(),
                            SimCameraProperties.PERFECT_90DEG().getDistCoeffs()
                    )
                    .withCalibrationError(
                            CameraProperties.Resolution.R640x480,
                            0.54,
                            0.06
                    )
                    .withFPS(
                            CameraProperties.Resolution.R640x480,
                            130
                    )
                    .withLatency(
                            CameraProperties.Resolution.R640x480,
                            6,
                            3
                    ),
            false
    );

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

    public double getStdDevFactor() { return stdDevFactor; }

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