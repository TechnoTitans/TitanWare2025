package frc.robot.subsystems.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.constants.Constants;
import frc.robot.constants.HardwareConstants;
import frc.robot.subsystems.drive.OdometryThreadRunner;
import frc.robot.subsystems.drive.SwerveModule;
import frc.robot.utils.logging.LogUtils;
import org.littletonrobotics.junction.Logger;

public class Gyro {
    protected static final String LogKey = "Gyro";

    private final GyroIO gyroIO;

    private final HardwareConstants.GyroConstants gyroConstants;
    private final GyroIOInputsAutoLogged inputs;
    private final boolean isReal;

    public Gyro(
            final HardwareConstants.GyroConstants gyroConstants,
            final OdometryThreadRunner odometryThreadRunner,
            final SwerveDriveKinematics kinematics,
            final SwerveModule[] swerveModules,
            final Constants.RobotMode mode
    ) {
        this.gyroConstants = gyroConstants;
        this.gyroIO = switch (mode) {
            case REAL -> new GyroIOPigeon2(gyroConstants, odometryThreadRunner);
            case SIM -> new GyroIOSim(gyroConstants, odometryThreadRunner, kinematics, swerveModules);
            case REPLAY, DISABLED -> new GyroIO() {
            };
        };

        this.inputs = new GyroIOInputsAutoLogged();
        this.isReal = mode == Constants.RobotMode.REAL;

        this.gyroIO.config();
    }

    public void updateInputs() {
        gyroIO.updateInputs(inputs);
    }

    public void periodic() {
        final double gyroPeriodicUpdateStart = RobotController.getFPGATime();

        gyroIO.periodic();
        Logger.processInputs(LogKey, inputs);

        Logger.recordOutput(
                LogKey + "/PeriodicIOPeriodMs",
                LogUtils.microsecondsToMilliseconds(RobotController.getFPGATime() - gyroPeriodicUpdateStart)
        );
    }

    public boolean hasHardwareFault() {
        return inputs.hasHardwareFault;
    }

    /**
     * Get the gyro constants
     *
     * @return the {@link HardwareConstants.GyroConstants}
     * @see HardwareConstants.GyroConstants
     */
    public HardwareConstants.GyroConstants getGyroConstants() {
        return gyroConstants;
    }

    /**
     * Get whether this Gyro is real (true/real if hardware exists, false if hardware does not exist - i.e. in a sim)
     *
     * @return true if the Gyro is real, false if not
     */
    public boolean isReal() {
        return isReal;
    }

    /**
     * Get the current yaw (heading) reported by the Gyro
     *
     * @return the current yaw (deg)
     */
    public double getYaw() {
        return inputs.yawPositionDeg;
    }

    /**
     * Get the current yaw (heading) as a {@link Rotation2d}
     *
     * @return the {@link Rotation2d} of the current yaw
     * @see Rotation2d
     */
    public Rotation2d getYawRotation2d() {
        return Rotation2d.fromDegrees(getYaw());
    }

    /**
     * Get the current yaw (heading) velocity (AngularVelocityZ) reported by the Gyro
     *
     * @return the current yaw velocity (deg/sec)
     */
    public double getYawVelocity() {
        return inputs.yawVelocityDegPerSec;
    }

    public double[] getOdometryTimestamps() {
        return inputs.odometryTimestampsSec;
    }

    public double[] getOdometryYawPositions() {
        return inputs.odometryYawPositionsDeg;
    }

    /**
     * Get the current yaw (heading) velocity (AngularVelocityZ) as a {@link Rotation2d}
     *
     * @return the {@link Rotation2d} of the current yaw velocity (deg/sec)
     * @see Rotation2d
     */
    public Rotation2d getYawVelocityRotation2d() {
        return Rotation2d.fromDegrees(getYawVelocity());
    }

    /**
     * Get the current pitch reported by the Gyro
     *
     * @return the current pitch (deg)
     */
    public double getPitch() {
        return inputs.pitchPositionDeg;
    }

    /**
     * Get the current pitch as a {@link Rotation2d}
     *
     * @return the {@link Rotation2d} of the current pitch
     * @see Rotation2d
     */
    public Rotation2d getPitchRotation2d() {
        return Rotation2d.fromDegrees(getPitch());
    }

    public double getPitchVelocity() {
        return inputs.pitchVelocityDegPerSec;
    }

    /**
     * Get the current roll reported by the Gyro
     *
     * @return the current roll (deg)
     */
    public double getRoll() {
        return inputs.rollPositionDeg;
    }

    /**
     * Get the current pitch as a {@link Rotation2d}
     *
     * @return the {@link Rotation2d} of the current pitch
     * @see Rotation2d
     */
    public Rotation2d getRollRotation2d() {
        return Rotation2d.fromDegrees(getRoll());
    }

    public double getRollVelocity() {
        return inputs.rollVelocityDegPerSec;
    }

    /**
     * See {@link GyroIO#setAngle(Rotation2d)}
     */
    public void setAngle(final Rotation2d angle) {
        gyroIO.setAngle(angle);
    }

    /**
     * See {@link GyroIO#zeroRotation()}
     */
    public void zeroRotation() {
        gyroIO.zeroRotation();
    }
}
