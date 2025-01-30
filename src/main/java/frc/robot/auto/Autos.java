package frc.robot.auto;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.ScoreCommands;
import frc.robot.constants.FieldConstants;
import frc.robot.state.GamepieceState;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.vision.PhotonVision;
import org.littletonrobotics.junction.Logger;

import java.util.List;
import java.util.Set;
import java.util.concurrent.atomic.AtomicInteger;

@SuppressWarnings("DuplicatedCode")
public class Autos {
    public static final String LogKey = "Auto";

    private final Swerve swerve;
    private final Superstructure superstructure;
    private final Intake intake;
    private final PhotonVision photonVision;

    private final ScoreCommands scoreCommands;
    private final GamepieceState gamepieceState;

    private final AutoFactory autoFactory;

    public Autos(
            final Swerve swerve,
            final Superstructure superstructure,
            final Intake intake,
            final PhotonVision photonVision,
            final ScoreCommands scoreCommands,
            final GamepieceState gamepieceState
    ) {
        this.swerve = swerve;
        this.superstructure = superstructure;
        this.intake = intake;
        this.photonVision = photonVision;

        this.scoreCommands = scoreCommands;
        this.gamepieceState = gamepieceState;

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

    private Command scoreAtLevel(
            final int reefSide,
            final ScoreCommands.ScorePosition scorePosition
    ) {
        return Commands.parallel(
                swerve.driveToPose(() -> FieldConstants.getBranchScoringPositions()
                        .get(reefSide)
                        .get(scorePosition.side())
                        .get(scorePosition.level().level)),
                Commands.sequence(
                        Commands.waitUntil(swerve.atHolonomicDrivePose),
                        Commands.deadline(
                                Commands.parallel(
                                        Commands.waitUntil(superstructure.atSuperstructureSetpoint),
                                        intake.scoreCoral()
                                ),
                                superstructure.toSuperstructureGoal(
                                        ScoreCommands.Level.LevelMap.get(scorePosition.level().level)
                                )
                        )
                )
        );
    }

    private Command intakeCoralFromHP() {
        return Commands.parallel(
                superstructure.toSuperstructureGoal(Superstructure.Goal.HP),
                intake.runCoralRollerVelocity(3)
        );
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

        final AtomicInteger scoreCounter = new AtomicInteger(0);
        final List<ScoreCommands.ScorePosition> reefScoringPositions = ScoreCommands.ReefScoringPositions;

        routine.active().onTrue(
                Commands.sequence(
                        Commands.runOnce(() -> {
                            scoreCounter.set(0);
                            intake.setCANRangeDistance(Units.inchesToMeters(6));
                        }),
                        cage0Reef5.resetOdometry(),
                        cage0Reef5.cmd()
                )
        );

        final Trigger atReef5FromStart = cage0Reef5.done();
        atReef5FromStart.onTrue(
                Commands.sequence(
                        swerve.wheelXCommand(),
                        Commands.defer(
                                        () -> scoreAtLevel(4, reefScoringPositions
                                                .get(
                                                        MathUtil.clamp(
                                                                scoreCounter.getAndAdd(1),
                                                                0,
                                                                reefScoringPositions.size()-1
                                                        )
                                                )),
                                        superstructure.getRequirements(intake, swerve)
                                )
                                .onlyIf(hasCoral),
                        reef5ToRightHP.cmd()
                )
        );

        final Trigger atHP = reef5ToRightHP.done();
        atHP.onTrue(
                Commands.sequence(
                        swerve.wheelXCommand(),
                        Commands.parallel(
                                intakeCoralFromHP().asProxy(),
                                Commands.waitUntil(hasCoral)
                                        .andThen(rightHPTOReef5.cmd())
                        )
                )
        );

        final Trigger atReef = rightHPTOReef5.done();
        atReef.onTrue(
                Commands.sequence(
                        Commands.defer(
                                        () -> scoreAtLevel(4, reefScoringPositions
                                                .get(
                                                        MathUtil.clamp(
                                                                scoreCounter.getAndAdd(1),
                                                                0,
                                                                reefScoringPositions.size()-1
                                                        )
                                                )),
                                        superstructure.getRequirements(intake, swerve)
                                )
                                .onlyIf(hasCoral),
                        reef5ToRightHP.cmd()
                )
        );

        return routine;
    }
}
