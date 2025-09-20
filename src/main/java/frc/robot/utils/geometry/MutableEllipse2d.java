package frc.robot.utils.geometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class MutableEllipse2d {
    private double x;
    private double y;
    private double theta;

    private Pose2d centerRef;

    private double xSemiAxis;
    private double ySemiAxis;

    public MutableEllipse2d(final Pose2d center, final double xSemiAxis, final double ySemiAxis) {
        this.x = center.getX();
        this.y = center.getY();
        this.theta = center.getRotation().getRadians();
        this.xSemiAxis = xSemiAxis;
        this.ySemiAxis = ySemiAxis;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getTheta() {
        return theta;
    }

    public Pose2d getCenter() {
        return centerRef;
    }

    public void setCenter(final Pose2d center) {
        this.x = center.getX();
        this.y = center.getY();
        this.theta = center.getRotation().getRadians();
        this.centerRef = center;
    }

    public void setCenter(final double x, final double y, final Rotation2d theta) {
        setCenter(new Pose2d(x, y, theta));
    }

    public double getXSemiAxis() {
        return xSemiAxis;
    }

    public double getYSemiAxis() {
        return ySemiAxis;
    }

    public void setAxes(final double xSemiAxis, final double ySemiAxis) {
        this.xSemiAxis = xSemiAxis;
        this.ySemiAxis = ySemiAxis;
    }
}
