package frc.robot.utils.geometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class Ellipse2dHelpers {
    /**
     * Solves the equation of an ellipse from the given point. This is a helper function used to
     * determine if that point lies inside of or on an ellipse.
     *
     * <pre>
     * (x - h)²/a² + (y - k)²/b² = 1
     * </pre>
     *
     * @param center The center of the ellipse.
     * @param xSemiAxis The X semi-axis of the ellipse.
     * @param ySemiAxis The Y semi-axis of the ellipse.
     * @param px The X coordinate of the point to solve for.
     * @param py The X coordinate of the point to solve for.
     * @return < 1.0 if the point lies inside the ellipse, == 1.0 if a point lies on the ellipse, and
     *     > 1.0 if the point lies outsides the ellipse.
     */
    private static double solveEllipseEquation(
            final Pose2d center,
            final double xSemiAxis,
            final double ySemiAxis,
            final double px,
            final double py
    ) {
        // Rotate the point by the inverse of the ellipse's rotation
        final double cx = center.getX();
        final double cy = center.getY();

        final Rotation2d theta = center.getRotation();
        final double cos = theta.getCos();
        final double sin = theta.getSin();

        final double rotatedPx = (px - cx) * cos - (py - cy) * sin + cx;
        final double rotatedPy = (px - cx) * sin + (py - cy) * cos + cy;

        final double x = rotatedPx - cx;
        final double y = rotatedPy - cy;

        return (x * x) / (xSemiAxis * xSemiAxis) + (y * y) / (ySemiAxis * ySemiAxis);
    }

    /**
     * Checks if a point is contained within this ellipse. This is inclusive, if the point lies on the
     * circumference this will return {@code true}.
     *
     * @param center The center of the ellipse.
     * @param xSemiAxis The X semi-axis of the ellipse.
     * @param ySemiAxis The Y semi-axis of the ellipse.
     * @param point The point to check.
     * @return True, if the point is within or on the ellipse.
     */
    public static boolean contains(
            final Pose2d center,
            final double xSemiAxis,
            final double ySemiAxis,
            final Translation2d point
    ) {
        return solveEllipseEquation(
                center,
                xSemiAxis,
                ySemiAxis,
                point.getX(),
                point.getY()
        ) <= 1.0;
    }

    /**
     * Checks if a point is contained within this ellipse. This is inclusive, if the point lies on the
     * circumference this will return {@code true}.
     *
     * @param center The center of the ellipse.
     * @param xSemiAxis The X semi-axis of the ellipse.
     * @param ySemiAxis The Y semi-axis of the ellipse.
     * @param x The X coordinate of the point to check.
     * @param y The Y coordinate of the point to check.
     * @return True, if the point is within or on the ellipse.
     */
    public static boolean contains(
            final Pose2d center,
            final double xSemiAxis,
            final double ySemiAxis,
            final double x,
            final double y
    ) {
        return solveEllipseEquation(
                center,
                xSemiAxis,
                ySemiAxis,
                x,
                y
        ) <= 1.0;
    }
}
