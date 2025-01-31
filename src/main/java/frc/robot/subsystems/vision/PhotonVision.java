package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.drive.constants.SwerveConstants;
import frc.robot.subsystems.vision.cameras.TitanCamera;
import frc.robot.subsystems.vision.estimator.VisionUpdate;
import frc.robot.utils.PoseUtils;
import frc.robot.utils.gyro.GyroUtils;
import frc.robot.utils.logging.LogUtils;
import frc.robot.utils.subsystems.VirtualSubsystem;
import org.littletonrobotics.junction.Logger;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

public class PhotonVision extends VirtualSubsystem {
    public static final String PhotonLogKey = "Vision";

    public static final double TranslationalVelocityTolerance = 1;
    public static final double AngularVelocityTolerance = 1;

    private final double maxLinearVelocity = SwerveConstants.Config.maxLinearVelocityMeterPerSec();
    private final double maxAngularVelocity = SwerveConstants.Config.maxAngularVelocityRadsPerSec();

    public static final AprilTagFieldLayout apriltagFieldLayout;

    static {
        apriltagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
        apriltagFieldLayout.setOrigin(AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide);
    }

    @SafeVarargs
    public static <T extends VisionIO> Map<T, VisionIO.VisionIOInputs> makeVisionIOInputsMap(
            final T... visionIOs
    ) {
        return Arrays.stream(visionIOs).collect(Collectors.toMap(
                photonVisionIO -> photonVisionIO,
                photonVisionIO -> new VisionIO.VisionIOInputs()
        ));
    }

    private final PhotonVisionRunner runner;
    private final Map<? extends VisionIO, VisionIO.VisionIOInputs> aprilTagVisionIOInputsMap;

    private final Swerve swerve;
    private final SwerveDrivePoseEstimator poseEstimator;
    private final Map<VisionIO, VisionUpdate> lastVisionUpdateMap;

    private double lastOdomReset = -1;

    public PhotonVision(
            final Constants.RobotMode robotMode,
            final Swerve swerve,
            final SwerveDrivePoseEstimator poseEstimator
    ) {
        this.runner = switch (robotMode) {
            case REAL -> new RealVisionRunner(
                    PhotonVision.apriltagFieldLayout,
                    PhotonVision.makeVisionIOInputsMap(
                            new RealVisionRunner.VisionIOApriltagReal(TitanCamera.PHOTON_FL_APRILTAG)
                    )
            );
            case SIM -> {
                final VisionSystemSim visionSystemSim = new VisionSystemSim(PhotonVision.PhotonLogKey);
                yield new SimVisionRunner(
                        swerve,
                        new SwerveDriveOdometry(
                                swerve.getKinematics(),
                                swerve.getYaw(),
                                swerve.getModulePositions(),
                                swerve.getPose()
                        ),
                        PhotonVision.apriltagFieldLayout,
                        visionSystemSim,
                        PhotonVision.makeVisionIOInputsMap(
                                new SimVisionRunner.VisionIOApriltagsSim(TitanCamera.PHOTON_FL_APRILTAG, visionSystemSim)
                        )
                );
            }
            case REPLAY -> new ReplayVisionRunner(
                    PhotonVision.apriltagFieldLayout,
                    PhotonVision.makeVisionIOInputsMap(
                            new ReplayVisionRunner.VisionIOReplay(TitanCamera.PHOTON_FL_APRILTAG)
                    )
            );
            case DISABLED -> new PhotonVisionRunner() {};
        };

        this.swerve = swerve;
        this.poseEstimator = poseEstimator;
        this.aprilTagVisionIOInputsMap = runner.getApriltagVisionIOInputsMap();

        this.lastVisionUpdateMap = new HashMap<>();
        final Pose2d estimatedPose = poseEstimator.getEstimatedPosition();
        resetPose(estimatedPose);
    }

    public enum EstimationRejectionReason {
        DID_NOT_REJECT(0),
        ESTIMATED_POSE_OBJECT_NULL(1),
        ESTIMATED_POSE_OR_TIMESTAMP_OR_TARGETS_INVALID(2),
        POSE_NOT_IN_FIELD(3),
        LAST_ESTIMATED_POSE_TIMESTAMP_INVALID_OR_TOO_CLOSE(4),
        POSE_IMPOSSIBLE_VELOCITY(5),
        FUTURE_TIMESTAMP(6),
        TIMESTAMP_OLDER_THEN_POSE_RESET(7);

        private final int id;
        EstimationRejectionReason(final int id) {
            this.id = id;
        }

        public int getId() {
            return id;
        }

        public static boolean wasRejected(final EstimationRejectionReason rejectionReason) {
            return rejectionReason != DID_NOT_REJECT;
        }

        public boolean wasRejected() {
            return wasRejected(this);
        }
    }

    public EstimationRejectionReason shouldRejectEstimation(
            final VisionUpdate lastVisionUpdate,
            final VisionUpdate visionUpdate
    ) {
        if (visionUpdate == null) {
            // reject immediately if the estimated pose itself is null
            return EstimationRejectionReason.ESTIMATED_POSE_OBJECT_NULL;
        }

        if (visionUpdate.estimatedPose() == null
                || visionUpdate.timestamp() == -1
                || visionUpdate.targetsUsed().isEmpty()) {
            // reject immediately if null estimatedPose, timestamp is invalid, or no targets used
            return EstimationRejectionReason.ESTIMATED_POSE_OR_TIMESTAMP_OR_TARGETS_INVALID;
        }

        if (lastVisionUpdate == null) {
            // do not reject if there was no last estimation at all (this is different from an invalid last estimation)
            // likely, this is the first time we have an estimation, make sure we accept this estimation
            return EstimationRejectionReason.DID_NOT_REJECT;
        }

        final Pose3d nextEstimatedPosition = visionUpdate.estimatedPose();

        if (!PoseUtils.isInField(nextEstimatedPosition)) {
//             reject if pose not within the field
            return EstimationRejectionReason.POSE_NOT_IN_FIELD;
        }

        final double secondsSinceLastUpdate =
                visionUpdate.timestamp() - lastVisionUpdate.timestamp();

        // TODO: this rejection showed up very often at event-cmp and didn't seem to help much,
        //  maybe re-evaluate why we added this rejection in the first place? (removed for now)
//        if (lastVisionUpdate.timestampSeconds == -1 || secondsSinceLastUpdate <= 0) {
        // TODO: do we always need to reject immediately here? maybe we can still use the next estimation even
        //  if the last estimation had no timestamp or was very close
//            return EstimationRejectionReason.LAST_ESTIMATED_POSE_TIMESTAMP_INVALID_OR_TOO_CLOSE;
//        }

        if (visionUpdate.timestamp() > Timer.getFPGATimestamp()) {
            return EstimationRejectionReason.FUTURE_TIMESTAMP;
        }

        if (visionUpdate.timestamp() <= lastOdomReset) {
            return EstimationRejectionReason.TIMESTAMP_OLDER_THEN_POSE_RESET;
        }

        // Only try calculating this rejection strategy if time > 0
        if (secondsSinceLastUpdate > 0) {
            final Pose2d nextEstimatedPosition2d = nextEstimatedPosition.toPose2d();
            final Pose2d lastEstimatedPosition2d = lastVisionUpdate.estimatedPose().toPose2d();
            final Twist2d twist2dToNewEstimation = lastEstimatedPosition2d.log(nextEstimatedPosition2d);

            final double xVel = twist2dToNewEstimation.dx / secondsSinceLastUpdate;
            final double yVel = twist2dToNewEstimation.dy / secondsSinceLastUpdate;
            final double thetaVel = twist2dToNewEstimation.dtheta / secondsSinceLastUpdate;
            final double translationVel = Math.hypot(xVel, yVel);

            Logger.recordOutput(PhotonLogKey + "/Rejection/TranslationVel", translationVel);
            Logger.recordOutput(PhotonLogKey + "/Rejection/ThetaVel", thetaVel);

            if ((Math.abs(translationVel) >= maxLinearVelocity + PhotonVision.TranslationalVelocityTolerance)
                    || (Math.abs(thetaVel) >= maxAngularVelocity + PhotonVision.AngularVelocityTolerance)) {
                // reject sudden pose changes resulting in an impossible velocity (cannot reach)
                return EstimationRejectionReason.POSE_IMPOSSIBLE_VELOCITY;
            }
        }

        return EstimationRejectionReason.DID_NOT_REJECT;
    }

    public Vector<N3> calculateStdDevs(final VisionUpdate visionUpdate, final double stdDevFactor) {
        if (visionUpdate.targetsUsed().isEmpty()) {
            return VecBuilder.fill(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
        }

        final int nTargetsUsed = visionUpdate.targetsUsed().size();
        double totalDistanceMeters = 0;
        for (final PhotonTrackedTarget target : visionUpdate.targetsUsed()) {
            totalDistanceMeters += target.getBestCameraToTarget().getTranslation().getNorm();
        }

        final double avgDistanceMeters = totalDistanceMeters / nTargetsUsed;
        return Constants.Vision.VISION_STD_DEV_COEFFS
                .times(Math.pow(avgDistanceMeters, 2))
                .div(nTargetsUsed)
                .times(stdDevFactor);
    }

    private void update() {
        for (
                final Map.Entry<? extends VisionIO, VisionIO.VisionIOInputs>
                        visionIOInputsEntry : aprilTagVisionIOInputsMap.entrySet()
        ) {
            final VisionIO visionIO = visionIOInputsEntry.getKey();
            final VisionIO.VisionIOInputs inputs = visionIOInputsEntry.getValue();
            final String logKey = PhotonLogKey + "/" + inputs.name;

            Logger.recordOutput(
                    logKey + "/CameraPose",
                    new Pose3d(swerve.getPose()).transformBy(inputs.robotToCamera)
            );

            final VisionUpdate visionUpdate = runner.getVisionUpdate(visionIO);
            if (visionUpdate != null) {
                final VisionUpdate lastVisionUpdate = lastVisionUpdateMap.get(visionIO);
                final EstimationRejectionReason rejectionReason =
                        shouldRejectEstimation(lastVisionUpdate, visionUpdate);

                Logger.recordOutput(logKey + "/RejectionReason", rejectionReason.getId());
                if (rejectionReason.wasRejected()) {
                    continue;
                }

                final Vector<N3> stdDevs = calculateStdDevs(visionUpdate, inputs.stdDevFactor);
                Logger.recordOutput(logKey + "/StdDevs", stdDevs.getData());

                lastVisionUpdateMap.put(visionIO, visionUpdate);
                Logger.recordOutput(logKey + "/LastVisionTimeStamp", visionUpdate.timestamp());
                poseEstimator.addVisionMeasurement(
                        visionUpdate.estimatedPose().toPose2d(),
                        visionUpdate.timestamp(),
                        stdDevs
                );
            }
        }
    }

    public void updateOutputs() {
        for (
                final Map.Entry<VisionIO, VisionUpdate>
                        visionUpdateEntry : lastVisionUpdateMap.entrySet()
        ) {
            final VisionIO.VisionIOInputs inputs = aprilTagVisionIOInputsMap.get(visionUpdateEntry.getKey());
            final VisionUpdate visionUpdate = visionUpdateEntry.getValue();

            final String logKey = PhotonVision.PhotonLogKey + "/" + inputs.name;
            if (visionUpdate == null) {
                continue;
            }

            final List<PhotonTrackedTarget> targetsUsed = visionUpdate.targetsUsed();
            final int nTargetsUsed = targetsUsed.size();

            final int[] apriltagIds = new int[nTargetsUsed];
            final Pose3d[] apriltagPose3ds = new Pose3d[nTargetsUsed];
            final Pose2d[] apriltagPose2ds = new Pose2d[nTargetsUsed];

            for (int i = 0; i < apriltagIds.length; i++) {
                final int fiducialId = targetsUsed.get(i).getFiducialId();
                final Pose3d tagPose3d = apriltagFieldLayout.getTagPose(fiducialId).orElseGet(Pose3d::new);
                final Pose2d tagPose2d = tagPose3d.toPose2d();

                apriltagIds[i] = fiducialId;
                apriltagPose3ds[i] = tagPose3d;
                apriltagPose2ds[i] = tagPose2d;
            }

            Logger.recordOutput(logKey + "/EstimatedPose3d", visionUpdate.estimatedPose());
            Logger.recordOutput(logKey + "/EstimatedPose2d", visionUpdate.estimatedPose().toPose2d());
            Logger.recordOutput(logKey + "/ApriltagIds", apriltagIds);

            Logger.recordOutput(logKey + "/ApriltagPose3ds", apriltagPose3ds);
            Logger.recordOutput(logKey + "/ApriltagPose2ds", apriltagPose2ds);
        }
    }

    @Override
    public void periodic() {
        final double visionIOPeriodicStart = RobotController.getFPGATime();
        runner.periodic(swerve.getPose());

        // Update and log PhotonVision results
        update();
        updateOutputs();

        Logger.recordOutput(
                PhotonLogKey + "/PeriodicIOPeriodMs",
                LogUtils.microsecondsToMilliseconds(RobotController.getFPGATime() - visionIOPeriodicStart)
        );
    }

    public void resetPose(final Pose2d robotPose, final Rotation2d robotYaw) {
        this.lastOdomReset = Timer.getFPGATimestamp();
        Logger.recordOutput("LastOdomResetTime", this.lastOdomReset);
        poseEstimator.resetPosition(robotYaw, swerve.getModulePositions(), robotPose);
        runner.resetRobotPose(GyroUtils.robotPose2dToPose3dWithGyro(
                new Pose2d(robotPose.getTranslation(), robotYaw),
                new Rotation3d(
                        swerve.getRoll().getRadians(),
                        swerve.getPitch().getRadians(),
                        swerve.getYaw().getRadians()
                )
        ));
    }

    public void resetPose(final Pose2d robotPose) {
        resetPose(robotPose, swerve.getYaw());
    }

    public Command resetPoseCommand(final Pose2d robotPose) {
        return runOnce(() -> resetPose(robotPose));
    }
}
