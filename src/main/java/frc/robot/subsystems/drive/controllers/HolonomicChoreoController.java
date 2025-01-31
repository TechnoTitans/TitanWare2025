package frc.robot.subsystems.drive.controllers;

import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class HolonomicChoreoController {
    private final PIDController xController;
    private final PIDController yController;
    private final PIDController rotationController;

    public HolonomicChoreoController(
            final PIDController xController,
            final PIDController yController,
            final PIDController rotationController
    ) {
        this.xController = xController;
        this.yController = yController;
        this.rotationController = rotationController;

        this.rotationController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public void reset() {
        xController.reset();
        yController.reset();
        rotationController.reset();
    }

    public ChassisSpeeds calculate(
            final Pose2d pose,
            final SwerveSample swerveSample
    ) {
        double xFF = swerveSample.vx;
        double yFF = swerveSample.vy;
        double rotationFF = swerveSample.omega;

        double xFeedback = xController.calculate(pose.getX(), swerveSample.x);
        double yFeedback = yController.calculate(pose.getY(), swerveSample.y);
        double rotationFeedback =
                rotationController.calculate(pose.getRotation().getRadians(), swerveSample.heading);

        return ChassisSpeeds.fromFieldRelativeSpeeds(
                xFF + xFeedback,
                yFF + yFeedback,
                rotationFeedback,
                pose.getRotation()
        );
    }
}
