package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class SplineProfile {
    public record BezierCurve(
            Translation2d p0,
            Translation2d p1,
            Translation2d p2,
            Translation2d p3
    ) {}

    public final double totalTime;

    public final Superstructure.Goal startingGoal;
    public final Superstructure.Goal endingGoal;

    public final double startElevatorArmAngleRads;
    public final double startElevatorExtensionMeters;
    public final double p1ElevatorArmAngleRads;
    public final double p1ElevatorExtensionMeters;
    public final double p2ElevatorArmAngleRads;
    public final double p2ElevatorExtensionMeters;
    public final double endElevatorArmAngleRads;
    public final double endElevatorExtensionMeters;

    private final BezierCurve bezierCurve;

    public SplineProfile(
            final double totalTime,
            final Superstructure.Goal startingGoal,
            final double p1ElevatorArmAngleRads,
            final double p1ElevatorExtensionMeters,
            final double p2ElevatorArmAngleRads,
            final double p2ElevatorExtensionMeters,
            final Superstructure.Goal endingGoal
    ) {
        this.totalTime = totalTime;

        this.startingGoal = startingGoal;
        this.endingGoal = endingGoal;

        this.startElevatorArmAngleRads =
                Units.rotationsToRadians(startingGoal.elevatorArmGoal.getPivotPositionGoalRots());
        this.startElevatorExtensionMeters = startingGoal.elevatorGoal.getPositionGoalMeters();
        this.p1ElevatorArmAngleRads = p1ElevatorArmAngleRads;
        this.p1ElevatorExtensionMeters = p1ElevatorExtensionMeters;
        this.p2ElevatorArmAngleRads = p2ElevatorArmAngleRads;
        this.p2ElevatorExtensionMeters = p2ElevatorExtensionMeters;
        this.endElevatorArmAngleRads =
                Units.rotationsToRadians(endingGoal.elevatorArmGoal.getPivotPositionGoalRots());
        this.endElevatorExtensionMeters = endingGoal.elevatorGoal.getPositionGoalMeters();

        this.bezierCurve = new BezierCurve(
                new Translation2d(startElevatorArmAngleRads, startElevatorExtensionMeters),
                new Translation2d(p1ElevatorArmAngleRads, p1ElevatorExtensionMeters),
                new Translation2d(p2ElevatorArmAngleRads, p2ElevatorExtensionMeters),
                new Translation2d(endElevatorArmAngleRads, endElevatorExtensionMeters)
        );
    }

    public BezierCurve getBezierCurve() {
        return bezierCurve;
    }

    public Supplier<Pose2d> sampler(final DoubleSupplier timeSupplier) {
        return () -> {
            final double time = timeSupplier.getAsDouble();
            final double alpha = MathUtil.clamp(time / totalTime, 0, 1);
            return sampleBezier(bezierCurve, alpha);
        };
    }

    //https://courses.grainger.illinois.edu/cs418/sp2009/notes/12-MoreSplines.pdf
    private static double bezierP(
            final double alpha,
            final double p0,
            final double p1,
            final double p2,
            final double p3
    ) {
        return Math.pow(1 - alpha, 3) * p0
                + 3 * Math.pow(1 - alpha, 2) * alpha * p1
                + 3 * (1 - alpha) * Math.pow(alpha, 2) * p2
                + Math.pow(alpha, 3) * p3;
    }

    public static Pose2d sampleBezier(final BezierCurve bezierCurve, final double alpha) {
        final Translation2d p0 = bezierCurve.p0();
        final Translation2d p1 = bezierCurve.p1();
        final Translation2d p2 = bezierCurve.p2();
        final Translation2d p3 = bezierCurve.p3();

        final double clamped = MathUtil.clamp(alpha, 0, 1);
        return new Pose2d(
                bezierP(clamped, p0.getX(), p1.getX(), p2.getX(), p3.getX()),
                bezierP(clamped, p0.getY(), p1.getY(), p2.getY(), p3.getY()),
                Rotation2d.kZero
        );
    }

    public static Pose2d[] getPosesAlongBezier(
            final BezierCurve bezierCurve,
            final double totalTime,
            final double timeStep
    ) {
        final Pose2d[] poses = new Pose2d[(int)(Math.floor(totalTime / timeStep) + 1)];
        for (int i = 0; i < Math.floor(totalTime / timeStep); i++) {
            final double time = i * timeStep;
            final Pose2d pose = SplineProfile.sampleBezier(bezierCurve, time / totalTime);
            poses[i] = pose;
        }
        poses[poses.length - 1] = SplineProfile.sampleBezier(bezierCurve, 1);

        return poses;
    }
}
