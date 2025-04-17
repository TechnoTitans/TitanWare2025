package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.drive.constants.SwerveConstants;
import frc.robot.subsystems.vision.cameras.TitanCamera;
import frc.robot.subsystems.vision.estimator.VisionResult;
import frc.robot.utils.PoseUtils;
import frc.robot.utils.gyro.GyroUtils;
import frc.robot.utils.logging.LogUtils;
import frc.robot.utils.subsystems.VirtualSubsystem;
import org.littletonrobotics.junction.Logger;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.io.IOException;
import java.util.*;
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
        try {
            apriltagFieldLayout = new AprilTagFieldLayout(
                    Filesystem.getDeployDirectory().getPath() + "/2025-reefscape-reef.json");
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
//        apriltagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
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
    private final Map<VisionIO, VisionResult> lastVisionUpdateMap;

    private double lastPoseResetTimestampSeconds = 0;

    public PhotonVision(
            final Constants.RobotMode robotMode,
            final Swerve swerve,
            final SwerveDrivePoseEstimator poseEstimator
    ) {
        this.runner = switch (robotMode) {
            case REAL -> new RealVisionRunner(
                    PhotonVision.apriltagFieldLayout,
                    PhotonVision.makeVisionIOInputsMap(
                            new RealVisionRunner.VisionIOApriltagReal(TitanCamera.PHOTON_FR_APRILTAG),
                            new RealVisionRunner.VisionIOApriltagReal(TitanCamera.PHOTON_BL_APRILTAG),
                            new RealVisionRunner.VisionIOApriltagReal(TitanCamera.PHOTON_FL_BOTTOM_APRILTAG)
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
                                new SimVisionRunner.VisionIOApriltagsSim(
                                        TitanCamera.PHOTON_FR_APRILTAG, visionSystemSim
                                ),
                                new SimVisionRunner.VisionIOApriltagsSim(
                                        TitanCamera.PHOTON_BL_APRILTAG, visionSystemSim
                                ),
                                new SimVisionRunner.VisionIOApriltagsSim(
                                        TitanCamera.PHOTON_FL_BOTTOM_APRILTAG, visionSystemSim
                                )
                        )
                );
            }
            case REPLAY -> new ReplayVisionRunner(
                    PhotonVision.apriltagFieldLayout,
                    PhotonVision.makeVisionIOInputsMap(
                            new ReplayVisionRunner.VisionIOReplay(TitanCamera.PHOTON_FR_APRILTAG),
                            new ReplayVisionRunner.VisionIOReplay(TitanCamera.PHOTON_BL_APRILTAG),
                            new ReplayVisionRunner.VisionIOReplay(TitanCamera.PHOTON_FL_BOTTOM_APRILTAG)
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

    public enum RejectionReason {
        DID_NOT_REJECT,
        ALREADY_REJECTED,
        ESTIMATED_POSE_OR_TIMESTAMP_OR_TARGETS_INVALID,
        POSE_NOT_IN_FIELD,
        POSE_IMPOSSIBLE_VELOCITY,
        FUTURE_TIMESTAMP,
        TIMESTAMP_OLDER_THEN_POSE_RESET;

        public static boolean wasRejected(final RejectionReason rejectionReason) {
            return rejectionReason != DID_NOT_REJECT;
        }

        public boolean wasRejected() {
            return wasRejected(this);
        }
    }

    public RejectionReason shouldReject(
            final VisionResult visionResult,
            final VisionResult lastVisionResult
    ) {
        final Optional<VisionResult.VisionUpdate> maybeVisionUpdate = visionResult.visionUpdate();
        if (maybeVisionUpdate.isEmpty()) {
            return RejectionReason.ALREADY_REJECTED;
        }

        final VisionResult.VisionUpdate visionUpdate = maybeVisionUpdate.get();
        final double visionTimestamp = visionUpdate.timestamp();
        if (visionUpdate.estimatedPose() == null
                || visionTimestamp == -1
                || visionUpdate.targetsUsed().isEmpty()) {
            // reject immediately if null estimatedPose, timestamp is invalid, or no targets used
            return RejectionReason.ESTIMATED_POSE_OR_TIMESTAMP_OR_TARGETS_INVALID;
        }

        final Pose3d estimatedPose = visionUpdate.estimatedPose();
        if (!PoseUtils.isInField(estimatedPose)) {
            // reject if pose not within the field
            return RejectionReason.POSE_NOT_IN_FIELD;
        }

        if (visionTimestamp > Timer.getTimestamp()) {
            return RejectionReason.FUTURE_TIMESTAMP;
        }

        if (visionTimestamp <= lastPoseResetTimestampSeconds) {
            return RejectionReason.TIMESTAMP_OLDER_THEN_POSE_RESET;
        }

        // do not try to reject if there was no last result at all (this is different from an invalid last result)
        // likely, this is the first time we have a result, make sure we accept this update
        if (lastVisionResult != null) {
            // there's a lot of nesting here, but it's probably a good idea to keep it this way instead of
            // moving out to guard-clauses because the only DID_NOT_REJECT return point should be at the end

            final Optional<VisionResult.VisionUpdate> maybeLastVisionUpdate = lastVisionResult.visionUpdate();
            if (maybeLastVisionUpdate.isPresent()) {
                final VisionResult.VisionUpdate lastVisionUpdate = maybeLastVisionUpdate.get();
                // Only try calculating this rejection strategy if delta-time > 0
                final double deltaTimeSinceLastUpdate = visionTimestamp - lastVisionUpdate.timestamp();
                if (deltaTimeSinceLastUpdate > 0) {
                    final Pose2d nextPose = estimatedPose.toPose2d();
                    final Pose2d lastPose = lastVisionUpdate.estimatedPose().toPose2d();
                    final Twist2d twist = lastPose.log(nextPose);

                    final double translationVelocity = Math.hypot(twist.dx, twist.dy) / deltaTimeSinceLastUpdate;
                    final double thetaVelocity = twist.dtheta / deltaTimeSinceLastUpdate;

                    // TODO: make better
                    Logger.recordOutput(PhotonLogKey + "/Rejection/TranslationVel", translationVelocity);
                    Logger.recordOutput(PhotonLogKey + "/Rejection/ThetaVel", thetaVelocity);

                    if ((Math.abs(translationVelocity) >= maxLinearVelocity + PhotonVision.TranslationalVelocityTolerance)
                            || (Math.abs(thetaVelocity) >= maxAngularVelocity + PhotonVision.AngularVelocityTolerance)) {
                        // reject sudden pose changes resulting in an impossible velocity (cannot reach)
                        return RejectionReason.POSE_IMPOSSIBLE_VELOCITY;
                    }
                }
            }
        }

        return RejectionReason.DID_NOT_REJECT;
    }

    public Vector<N3> calculateStdDevs(
            final VisionResult.VisionUpdate visionUpdate,
            final Set<Integer> oppositeReefTagIds,
            final double stdDevFactor,
            final double oppositeReefStdFactor
    ) {
        if (visionUpdate.targetsUsed().isEmpty()) {
            return VecBuilder.fill(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
        }

        final int nTargetsUsed = visionUpdate.targetsUsed().size();
        double totalDistanceMeters = 0;
        for (final PhotonTrackedTarget target : visionUpdate.targetsUsed()) {
            totalDistanceMeters += target.getBestCameraToTarget().getTranslation().getNorm();
        }

        double stdDev = stdDevFactor;
        if (MathUtil.isNear(stdDevFactor, oppositeReefStdFactor, 1E-6)) {
            for (final PhotonTrackedTarget target : visionUpdate.targetsUsed()) {
                if (oppositeReefTagIds.contains(target.getFiducialId())) {
                    stdDev = oppositeReefStdFactor;
                    break;
                }
            }
        }

        final double avgDistanceMeters = totalDistanceMeters / nTargetsUsed;
        return Constants.Vision.VISION_STD_DEV_COEFFS
                .times(Math.pow(avgDistanceMeters, 2))
                .div(nTargetsUsed)
                .times(stdDev);
    }

    private void update() {
        final Set<Integer> oppositeReefTagIds = Robot.IsRedAlliance.getAsBoolean()
                ? FieldConstants.Reef.BLUE_APRILTAG_IDS
                : FieldConstants.Reef.RED_APRILTAG_IDS;

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

            final VisionResult visionResult = runner.getVisionResult(visionIO);
            if (visionResult != null) {
                final VisionResult lastVisionResult = lastVisionUpdateMap.get(visionIO);
                final RejectionReason rejectionReason =
                        shouldReject(visionResult, lastVisionResult);
                final boolean rejected = rejectionReason.wasRejected();

                Logger.recordOutput(logKey + "/Rejected", rejected);
                Logger.recordOutput(logKey + "/RejectionReason", rejectionReason);
                Logger.recordOutput(logKey + "/VisionResult", visionResult.result());

                final Optional<VisionResult.VisionUpdate> maybeVisionUpdate = visionResult.visionUpdate();
                if (maybeVisionUpdate.isEmpty()) {
                    continue;
                }

                final VisionResult.VisionUpdate visionUpdate = maybeVisionUpdate.get();
                final double visionUpdateTimestamp = visionUpdate.timestamp();
                Logger.recordOutput(logKey + "/VisionUpdateTimestamp", visionUpdateTimestamp);

                if (rejected) {
                    continue;
                }

                final Vector<N3> stdDevs = calculateStdDevs(
                        visionUpdate,
                        oppositeReefTagIds,
                        inputs.stdDevFactor,
                        inputs.oppositeReefStdFactor
                );
                Logger.recordOutput(logKey + "/StdDevs", stdDevs.getData());

                lastVisionUpdateMap.put(visionIO, visionResult);
                poseEstimator.addVisionMeasurement(
                        visionUpdate.estimatedPose().toPose2d(),
                        visionUpdateTimestamp,
                        stdDevs
                );
            }
        }
    }

    public void updateOutputs() {
        Logger.recordOutput(PhotonLogKey + "/LastPoseResetTimestampSeconds", this.lastPoseResetTimestampSeconds);

        for (
                final Map.Entry<VisionIO, VisionResult>
                        visionUpdateEntry : lastVisionUpdateMap.entrySet()
        ) {
            final VisionIO.VisionIOInputs inputs = aprilTagVisionIOInputsMap.get(visionUpdateEntry.getKey());
            final VisionResult visionResult = visionUpdateEntry.getValue();
            final Optional<VisionResult.VisionUpdate> maybeVisionUpdate = visionResult.visionUpdate();

            final String logKey = PhotonVision.PhotonLogKey + "/" + inputs.name;

            final int nTargetsUsed = maybeVisionUpdate
                    .map(update -> update.targetsUsed().size())
                    .orElse(0);

            final int[] apriltagIds = new int[nTargetsUsed];
            final Pose3d[] apriltagPose3ds = new Pose3d[nTargetsUsed];
            final Pose2d[] apriltagPose2ds = new Pose2d[nTargetsUsed];

            if (maybeVisionUpdate.isPresent()) {
                final VisionResult.VisionUpdate visionUpdate = maybeVisionUpdate.get();
                final Pose3d estimatedPose = visionUpdate.estimatedPose();
                final Pose3d ambiguousPose = visionUpdate.ambiguousPose();

                final List<PhotonTrackedTarget> targetsUsed = visionUpdate.targetsUsed();
                for (int i = 0; i < apriltagIds.length; i++) {
                    final int fiducialId = targetsUsed.get(i).getFiducialId();
                    final Pose3d tagPose3d = apriltagFieldLayout.getTagPose(fiducialId).orElseGet(Pose3d::new);
                    final Pose2d tagPose2d = tagPose3d.toPose2d();

                    apriltagIds[i] = fiducialId;
                    apriltagPose3ds[i] = tagPose3d;
                    apriltagPose2ds[i] = tagPose2d;
                }

                Logger.recordOutput(logKey + "/EstimatedPose3d", estimatedPose);
                Logger.recordOutput(logKey + "/AmbiguousPose3d", ambiguousPose);
                Logger.recordOutput(logKey + "/EstimatedPose2d", estimatedPose.toPose2d());
            }

            Logger.recordOutput(logKey + "/TagIds", apriltagIds);

            Logger.recordOutput(logKey + "/TagPose3ds", apriltagPose3ds);
            Logger.recordOutput(logKey + "/TagPose2ds", apriltagPose2ds);
        }
    }

    @Override
    public void periodic() {
        final double visionIOPeriodicStart = RobotController.getFPGATime();
        runner.periodic(swerve::getPose);

        // Update and log PhotonVision results
        update();
        updateOutputs();

        Logger.recordOutput(
                PhotonLogKey + "/PeriodicIOPeriodMs",
                LogUtils.microsecondsToMilliseconds(RobotController.getFPGATime() - visionIOPeriodicStart)
        );
    }

    public void resetPose(final Pose2d robotPose) {
        poseEstimator.resetPose(robotPose);
        runner.resetRobotPose(GyroUtils.robotPose2dToPose3dWithGyro(
                new Pose2d(robotPose.getTranslation(), robotPose.getRotation()),
                new Rotation3d(
                        swerve.getRoll().getRadians(),
                        swerve.getPitch().getRadians(),
                        swerve.getYaw().getRadians()
                )
        ));

        this.lastPoseResetTimestampSeconds = Timer.getTimestamp();
    }

    @SuppressWarnings("unused")
    public Command resetPoseCommand(final Pose2d robotPose) {
        return runOnce(() -> resetPose(robotPose));
    }
}
