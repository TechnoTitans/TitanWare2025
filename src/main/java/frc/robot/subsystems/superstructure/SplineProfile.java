package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.math.spline.CubicHermiteSpline;
import edu.wpi.first.math.spline.Spline;

import java.util.ArrayList;
import java.util.List;

public class SplineProfile {
    private static final Matrix<N4, N4> bezierToHermiteBasis = MatBuilder.fill(
            Nat.N4(), Nat.N4(),
            1, 0, 0, 0,
            0, 0, 0, 1,
            -3, 3, 0, 0,
            0, 0, -3, 3
    );

    private static final Matrix<N4, N4> bezierBases = MatBuilder.fill(
            Nat.N4(), Nat.N4(),
            1, 0, 0, 0,
            -3, 3, 0, 0,
            3, -6, 3, 0,
            -1, 3, -3, 1
    );

    public record BezierCurve(
            Translation2d p0,
            Translation2d p1,
            Translation2d p2,
            Translation2d p3
    ) {}

    public final double totalTime;

    public final double startElevatorArmAngleRads;
    public final double startElevatorExtensionMeters;
    public final double p1ElevatorArmAngleRads;
    public final double p1ElevatorExtensionMeters;
    public final double p2ElevatorArmAngleRads;
    public final double p2ElevatorExtensionMeters;
    public final double endElevatorArmAngleRads;
    public final double endElevatorExtensionMeters;

    private final BezierCurve bezierCurve;
    private final CubicHermiteSpline cubicHermiteSpline;

    public SplineProfile(
            final double totalTime,
            final double startElevatorArmAngleRads,
            final double startElevatorExtensionMeters,
            final double p1ElevatorArmAngleRads,
            final double p1ElevatorExtensionMeters,
            final double p2ElevatorArmAngleRads,
            final double p2ElevatorExtensionMeters,
            final double endElevatorArmAngleRads,
            final double endElevatorExtensionMeters
    ) {
        this.totalTime = totalTime;

        this.startElevatorArmAngleRads = startElevatorArmAngleRads;
        this.startElevatorExtensionMeters = startElevatorExtensionMeters;
        this.p1ElevatorArmAngleRads = p1ElevatorArmAngleRads;
        this.p1ElevatorExtensionMeters = p1ElevatorExtensionMeters;
        this.p2ElevatorArmAngleRads = p2ElevatorArmAngleRads;
        this.p2ElevatorExtensionMeters = p2ElevatorExtensionMeters;
        this.endElevatorArmAngleRads = endElevatorArmAngleRads;
        this.endElevatorExtensionMeters = endElevatorExtensionMeters;

        this.bezierCurve = new BezierCurve(
                new Translation2d(startElevatorArmAngleRads, startElevatorExtensionMeters),
                new Translation2d(p1ElevatorArmAngleRads, p1ElevatorExtensionMeters),
                new Translation2d(p2ElevatorArmAngleRads, p2ElevatorExtensionMeters),
                new Translation2d(endElevatorArmAngleRads, endElevatorExtensionMeters)
        );

        this.cubicHermiteSpline = SplineProfile.bezierCurveToCubicHermiteSpline(bezierCurve);
    }

    public BezierCurve getBezierCurve() {
        return bezierCurve;
    }

    public CubicHermiteSpline getCubicHermiteSpline() {
        return cubicHermiteSpline;
    }

    public static Pose2d[] getPosesAlongBezier(
            final BezierCurve bezierCurve,
            final double totalTime,
            final double timeStep
    ) {
        final Translation2d p0 = bezierCurve.p0();
        final Translation2d p1 = bezierCurve.p1();
        final Translation2d p2 = bezierCurve.p2();
        final Translation2d p3 = bezierCurve.p3();

        final List<Pose2d> poses = new ArrayList<>();
        for (double time = 0; time <= totalTime; time += timeStep) {
            final double clampedTime = MathUtil.clamp(time/totalTime, 0, 1);

            final double x =
                    Math.pow(1 - clampedTime, 3) * p0.getX()
                            + 3 * Math.pow(1 - clampedTime, 2) * clampedTime * p1.getX()
                            + 3 * (1 - clampedTime) * Math.pow(clampedTime, 2) * p2.getX()
                            + Math.pow(clampedTime, 3) * p3.getX();

            final double y =
                    Math.pow(1 - clampedTime, 3) * p0.getY()
                            + 3 * Math.pow(1 - clampedTime, 2) * clampedTime * p1.getY()
                            + 3 * (1 - clampedTime) * Math.pow(clampedTime, 2) * p2.getY()
                            + Math.pow(clampedTime, 3) * p3.getY();

            poses.add(new Pose2d(x, y, Rotation2d.kZero));
        }

        return poses.toArray(Pose2d[]::new);

//        final Matrix<N4, N1> bezierXCoeffs = bezierBases.times(
//                MatBuilder.fill(
//                        Nat.N4(), Nat.N1(),
//                        p0.getX(),
//                        p1.getX(),
//                        p2.getX(),
//                        p3.getX()
//                )
//        );
//        final Matrix<N4, N1> bezierYCoeffs = bezierBases.times(
//                MatBuilder.fill(
//                        Nat.N4(), Nat.N1(),
//                        p0.getY(),
//                        p1.getY(),
//                        p2.getY(),
//                        p3.getY()
//                )
//        );
//
//        final List<Pose2d> poses = new ArrayList<>();
//        for (double time = 0; time <= totalTime; time += timeStep) {
//            final double clampedTime = MathUtil.clamp(time/totalTime, 0, 1);
//            final Matrix<N1, N4> times = MatBuilder.fill(
//                    Nat.N1(), Nat.N4(),
//                    1, clampedTime, Math.pow(clampedTime, 2), Math.pow(clampedTime, 3)
//            );
//            final Matrix<N1, N1> bezierX = times.times(bezierXCoeffs);
//            final Matrix<N1, N1> bezierY = times.times(bezierYCoeffs);
//
//            poses.add(
//                    new Pose2d(
//                            bezierX.get(0, 0),
//                            bezierY.get(0, 0),
//                            Rotation2d.kZero
//                    )
//            );
//        }
//
//        return poses.toArray(Pose2d[]::new);
    }

    public static CubicHermiteSpline bezierCurveToCubicHermiteSpline(final BezierCurve bezierCurve) {
        final Translation2d p0 = bezierCurve.p0();
        final Translation2d p1 = bezierCurve.p1();
        final Translation2d p2 = bezierCurve.p2();
        final Translation2d p3 = bezierCurve.p3();

        final double p0X = p0.getX();
        final double p3X = p3.getX();

        final double p1X = p1.getX();
        final double p2X = p2.getX();
        final double r0X = 3 * (p1X - p0X);
        final double r3x = 3 * (p3X - p2X);

        final double p0Y = p0.getY();
        final double p3Y = p3.getY();

        final double p1Y = p1.getY();
        final double p2Y = p2.getY();
        final double r0Y = 3 * (p1Y - p0Y);
        final double r3Y = 3 * (p3Y - p2Y);

        final Spline.ControlVector initial = new Spline.ControlVector(
                new double[] {p0X, r0X},
                new double[] {p0Y, r0Y}
        );
        final Spline.ControlVector end = new Spline.ControlVector(
                new double[] {p3X, r3x},
                new double[] {p3Y, r3Y}
        );

        return new CubicHermiteSpline(
                initial.x,
                end.x,
                initial.y,
                end.y
        );

//        final Matrix<N4, N1> cubicHermiteXCoeffs = bezierToHermiteBasis.times(
//                MatBuilder.fill(
//                        Nat.N4(), Nat.N1(),
//                        p0.getX(),
//                        p1.getX(),
//                        p2.getX(),
//                        p3.getX()
//                )
//        );
//        final Matrix<N4, N1> cubicHermiteYCoeffs = bezierToHermiteBasis.times(
//                MatBuilder.fill(
//                        Nat.N4(), Nat.N1(),
//                        p0.getY(),
//                        p1.getY(),
//                        p2.getY(),
//                        p3.getY()
//                )
//        );
//
//        final Spline.ControlVector start = new Spline.ControlVector(
//                new double[] {cubicHermiteXCoeffs.get(0, 0), cubicHermiteXCoeffs.get(2, 0)},
//                new double[] {cubicHermiteYCoeffs.get(0, 0), cubicHermiteYCoeffs.get(2, 0)}
//        );
//        final Spline.ControlVector end = new Spline.ControlVector(
//                new double[] {cubicHermiteXCoeffs.get(1, 0), cubicHermiteXCoeffs.get(3, 0)},
//                new double[] {cubicHermiteYCoeffs.get(1, 0), cubicHermiteYCoeffs.get(3, 0)}
//        );
//
//        return new CubicHermiteSpline(
//                start.x,
//                end.x,
//                start.y,
//                end.y
//        );
    }
}
