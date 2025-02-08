package frc.robot.auto;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.ScoreCommands;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.FieldConstants.Reef;
import frc.robot.state.GamepieceState;
import frc.robot.state.ReefState;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.vision.PhotonVision;
import org.littletonrobotics.junction.Logger;

import java.util.HashSet;
import java.util.List;
import java.util.Set;

@SuppressWarnings("DuplicatedCode")
public class Autos {
    public static final String LogKey = "Auto";

    private final Swerve swerve;
    private final Superstructure superstructure;
    private final Intake intake;
    private final PhotonVision photonVision;

    private final ScoreCommands scoreCommands;
    private final ReefState reefState;
    private final GamepieceState gamepieceState;

    private final AutoFactory autoFactory;

    public Autos(
            final Swerve swerve,
            final Superstructure superstructure,
            final Intake intake,
            final PhotonVision photonVision,
            final ScoreCommands scoreCommands,
            final GamepieceState gamepieceState,
            final ReefState reefState
    ) {
        this.swerve = swerve;
        this.superstructure = superstructure;
        this.intake = intake;
        this.photonVision = photonVision;

        this.scoreCommands = scoreCommands;
        this.gamepieceState = gamepieceState;
        this.reefState = reefState;

        this.autoFactory = new AutoFactory(
                swerve::getPose,
                photonVision::resetPose,
                swerve::followChoreoSample,
                true,
                swerve,
                (trajectory, trajectoryRunning) -> {
                    Logger.recordOutput(
                            Autos.LogKey + "/Trajectory/Path",
                            (Robot.IsRedAlliance.getAsBoolean() ? trajectory.flipped() : trajectory).getPoses()
                    );

                    Logger.recordOutput(
                            Autos.LogKey + "/Trajectory/Name",
                            trajectory.name()
                    );

                    Logger.recordOutput(
                            Autos.LogKey + "/Trajectory/Running",
                            trajectoryRunning
                    );
                }
        );
    }

    private Command scoreAtLevel(final ReefState.Branch branch) {
        return Commands.parallel(
                swerve.driveToPose(() -> FieldConstants.getBranchScoringPositions()
                        .get(branch.face())
                        .get(branch.side())
                        .get(branch.level())),
                Commands.sequence(
                        Commands.waitUntil(swerve.atHolonomicDrivePose),
                        Commands.deadline(
                                Commands.parallel(
                                        Commands.waitUntil(superstructure.atSuperstructureSetpoint),
                                        intake.scoreCoral()
                                ),
                                superstructure.toSuperstructureGoal(
                                        ScoreCommands.Level.LevelMap.get(branch.level())
                                )
                        ),
                        Commands.waitUntil(superstructure.atSuperstructureSetpoint)
                )
        );
    }

    private Command intakeCoralFromHP() {
        return Commands.parallel(
                superstructure.toSuperstructureGoal(Superstructure.Goal.HP),
                intake.intakeCoralHP()
        );
    }

    private Command faceClosestHP() {
        return swerve.faceAngle(() -> {
            final Pose2d currentPose = swerve.getPose();
            final Pose2d nearestStation;
            if (Robot.IsRedAlliance.getAsBoolean()) {
                nearestStation = currentPose.nearest(
                        FieldConstants.CoralStation.RED_CORAL_STATIONS
                );
            } else {
                nearestStation = currentPose.nearest(
                        FieldConstants.CoralStation.BLUE_CORAL_STATIONS
                );
            }
            return nearestStation.getRotation().rotateBy(Rotation2d.kPi);
        });
    }

    public Set<Subsystem> getRequirements() {
        final Set<Subsystem> requirements = new HashSet<>(superstructure.getRequirements());
        requirements.addAll(List.of(swerve, intake));
        return requirements;
    }

    public AutoRoutine doNothing() {
        final AutoRoutine routine = autoFactory.newRoutine("DoNothing");

        routine.active().whileTrue(
                Commands.waitUntil(() -> !DriverStation.isAutonomousEnabled())
        );

        return routine;
    }

    public AutoRoutine cage0ToReef5() {
        final AutoRoutine routine = autoFactory.newRoutine("Cage0ToReef5");
        final AutoTrajectory cage0Reef5 = routine.trajectory("Cage0Reef5");
        final AutoTrajectory reef5ToRightHP = routine.trajectory("Reef5ToRightHP");
        final AutoTrajectory rightHPTOReef5 = routine.trajectory("RightHPTOReef5");

        final Trigger hasCoral = routine.observe(gamepieceState.hasCoral);

        routine.active().onTrue(
                Commands.sequence(
                        Commands.runOnce(() -> intake.setCANRangeDistance(Units.inchesToMeters(6))),
                        cage0Reef5.resetOdometry(),
                        cage0Reef5.cmd()
                )
        );

        final Trigger atReef5FromStart = cage0Reef5.done();
        atReef5FromStart.onTrue(
                Commands.sequence(
                        swerve.wheelXCommand(),
                        Commands.defer(
                                () -> reefState.getAndSetNextBranch(Reef.Face.FOUR)
                                        .map(this::scoreAtLevel)
                                        .orElseGet(Commands::idle),
                                getRequirements()
                        ).onlyIf(hasCoral),
                        reef5ToRightHP.cmd()
                )
        );

        final Trigger atHP = reef5ToRightHP.done();
        atHP.onTrue(
                Commands.parallel(
                        intakeCoralFromHP().asProxy(),
                        faceClosestHP(),
                        Commands.sequence(
                                Commands.waitUntil(hasCoral),
                                rightHPTOReef5.cmd().asProxy()
                        )
                )
        );

        final Trigger atReef = rightHPTOReef5.done();
        atReef.onTrue(
                Commands.sequence(
                        Commands.defer(
                                () -> reefState.getAndSetNextBranch(Reef.Face.FOUR)
                                        .map(this::scoreAtLevel)
                                        .orElseGet(Commands::idle),
                                getRequirements()
                        ).onlyIf(hasCoral),
                        reef5ToRightHP.cmd()
                )
        );

        return routine;
    }
}
