package frc.robot.subsystems.drive.constants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.Constants;
import frc.robot.constants.RobotMap;
import frc.robot.subsystems.drive.OdometryThreadRunner;
import frc.robot.subsystems.drive.SwerveModule;

public class SwerveConstants {
    private static final DCMotor KrakenX60Foc = DCMotor.getKrakenX60Foc(1);

    public static final SwerveConfig Config = new SwerveConfig(
            0.0508,
            5.9,
            150.0 / 7.0,
            50.0 / 16.0,
            Units.inchesToMeters(24),
            Units.inchesToMeters(24),
            Units.feetToMeters(17.1),
            Units.feetToMeters(80.0),
            12,
            6,
            new Translation2d(0, 0)
    );

    public static final SwerveModuleConstants FrontLeftModule = new SwerveModuleConstants(
            "FrontLeft",
            RobotMap.CanivoreCANBus,
            new Translation2d(Config.wheelBaseMeters / 2, Config.trackWidthMeters / 2),
            1,
            2,
            3,
            0.083984375,
            SwerveConstants.KrakenX60Foc.KtNMPerAmp
    );

    public static final SwerveModuleConstants FrontRightModule = new SwerveModuleConstants(
            "FrontRight",
            RobotMap.CanivoreCANBus,
            new Translation2d(Config.wheelBaseMeters / 2, -Config.trackWidthMeters / 2),
            4,
            5,
            6,
            -0.421875,
            SwerveConstants.KrakenX60Foc.KtNMPerAmp
    );

    public static final SwerveModuleConstants BackLeftModule = new SwerveModuleConstants(
            "BackLeft",
            RobotMap.CanivoreCANBus,
            new Translation2d(-Config.wheelBaseMeters / 2, Config.trackWidthMeters / 2),
            7,
            8,
            9,
            0.048828125,
            SwerveConstants.KrakenX60Foc.KtNMPerAmp
    );

    public static final SwerveModuleConstants BackRightModule = new SwerveModuleConstants(
            "BackRight",
            RobotMap.CanivoreCANBus,
            new Translation2d(-Config.wheelBaseMeters / 2, -Config.trackWidthMeters / 2),
            10,
            11,
            12,
            0.28515625,
            SwerveConstants.KrakenX60Foc.KtNMPerAmp
    );

    public record SwerveConfig(
            double wheelRadiusMeters,
            double driveReduction,
            double turnReduction,
            double couplingRatio,
            double wheelBaseMeters,
            double trackWidthMeters,
            double maxLinearVelocityMeterPerSec,
            double maxLinearAccelerationMetersPerSecSquared,
            double maxAngularVelocityRadsPerSec,
            double maxAngularAccelerationRadsPerSecSquared,
            Translation2d centerOfRotationMeters
    ) {
        public double driveBaseRadiusMeters() {
            return Math.hypot(wheelBaseMeters / 2, trackWidthMeters / 2);
        }

        public double wheelCircumferenceMeters() {
            return 2 * Math.PI * wheelRadiusMeters;
        }
    }

    public record SwerveModuleConstants(
            String name,
            String moduleCANBus,
            Translation2d translationOffset,
            int driveMotorId,
            int turnMotorId,
            int turnEncoderId,
            double turnEncoderOffsetRots,
            double driveMotorKtNmPerAmp
    ) {
        public static SwerveModule create(
                final SwerveModuleConstants constants,
                final OdometryThreadRunner odometryThreadRunner,
                final Constants.RobotMode robotMode
        ) {
            return new SwerveModule(constants, odometryThreadRunner, robotMode);
        }

        public SwerveModule create(
                final Constants.RobotMode robotMode,
                final OdometryThreadRunner odometryThreadRunner
        ) {
            return SwerveModuleConstants.create(this, odometryThreadRunner, robotMode);
        }
    }
}
