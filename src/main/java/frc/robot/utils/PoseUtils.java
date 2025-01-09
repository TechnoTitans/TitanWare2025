package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.constants.FieldConstants;

public class PoseUtils {
    public static final double FIELD_WITHIN_BORDER_MARGIN = 0.5;
    public static final double FIELD_WITHIN_Z_MARGIN = 0.75;

    private PoseUtils() {
    }

    // TODO: verify that this actually works how we think it works
    public static Pose2d flip(final Pose2d pose2d) {
        final Rotation2d rotation = pose2d.getRotation();
        return new Pose2d(
                FieldConstants.FIELD_LENGTH_X_METERS - pose2d.getX(),
                pose2d.getY(),
                new Rotation2d(-rotation.getCos(), rotation.getSin())
        );
    }

    public static boolean isInField(final Pose3d pose3d) {
        return pose3d.getX() >= -FIELD_WITHIN_BORDER_MARGIN
                && pose3d.getX() <= FieldConstants.FIELD_LENGTH_X_METERS + FIELD_WITHIN_BORDER_MARGIN
                && pose3d.getY() >= -FIELD_WITHIN_BORDER_MARGIN
                && pose3d.getY() <= FieldConstants.FIELD_WIDTH_Y_METERS + FIELD_WITHIN_BORDER_MARGIN
                && pose3d.getZ() >= -FIELD_WITHIN_Z_MARGIN
                && pose3d.getZ() <= FIELD_WITHIN_Z_MARGIN;
    }
}
