package frc.robot.auto;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.vision.PhotonVision;
import org.littletonrobotics.junction.Logger;

@SuppressWarnings("DuplicatedCode")
public class Autos {
    public static final String LogKey = "Auto";

    private final Swerve swerve;
    private final PhotonVision photonVision;
    private final AutoFactory autoFactory;

    public Autos(
            final Swerve swerve,
            final PhotonVision photonVision
    ) {
        this.swerve = swerve;
        this.photonVision = photonVision;

        this.autoFactory = new AutoFactory(
            swerve::getPose,
            photonVision::resetPose,
            swerve::followChoreoSample,
            true,
            swerve,
            (trajectory, trajectoryRunning) -> {
                Logger.recordOutput(
                    Autos.LogKey + "/Trajectory",
                        (Robot.IsRedAlliance.getAsBoolean() ? trajectory.flipped() : trajectory).getPoses()
                );

                Logger.recordOutput(
                    Autos.LogKey + "/TrajectoryRunning",
                    trajectoryRunning
                );
            }
        );
    }

    public AutoRoutine doNothing() {
        final AutoRoutine routine = autoFactory.newRoutine("DoNothing");

        routine.active().whileTrue(
                Commands.waitUntil(() -> !DriverStation.isAutonomousEnabled())
        );

        return routine;
    }
}
