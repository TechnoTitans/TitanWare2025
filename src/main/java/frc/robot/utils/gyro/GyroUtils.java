package frc.robot.utils.gyro;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Twist3d;
import frc.robot.subsystems.drive.constants.SwerveConstants;

public class GyroUtils {
    private static final double wheelBaseMeters = SwerveConstants.Config.wheelBaseMeters();
    private static final double trackWidthMeters = SwerveConstants.Config.trackWidthMeters();

    public static Pose3d robotPose2dToPose3dWithGyro(final Pose2d pose2d, final Rotation3d gyroRotation) {
        return new Pose3d(pose2d)
                .exp(new Twist3d(
                        0, 0, Math.abs(gyroRotation.getY()) * wheelBaseMeters * 0.5,
                        0, gyroRotation.getY(), 0
                ))
                .exp(new Twist3d(
                        0, 0, Math.abs(gyroRotation.getX()) * trackWidthMeters * 0.5,
                        gyroRotation.getX(), 0, 0
                ));
    }
}
