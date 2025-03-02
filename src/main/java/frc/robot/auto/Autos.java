package frc.robot.auto;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
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

// C:\Users\TechnoTitans\AppData\Local\Choreo>choreo-cli.exe --chor C:\Users\TechnoTitans\Documents\Repos\TitanWare2025\src\main\deploy\choreo\choreo.chor --all-trajectory --generate
public class Autos {
    public static final String LogKey = "Auto";

    private final Swerve swerve;
    private final Superstructure superstructure;
    private final Intake intake;

    private final ScoreCommands scoreCommands;
    private final GamepieceState gamepieceState;
    private final ReefState reefState;

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
                Commands.sequence(
                        swerve.driveToPose(() -> {
                            final Pose2d scoringPose = FieldConstants.getBranchScoringPositions()
                                    .get(branch.face())
                                    .get(branch.side())
                                    .get(branch.level());

                            return scoreCommands.offsetScoringPoseWithCANRange(scoringPose);
                        }),
                        swerve.wheelXCommand()
                ),
                Commands.sequence(
                        Commands.waitUntil(swerve.atHolonomicDrivePose).withTimeout(5),
                        Commands.deadline(
                                Commands.sequence(
                                        Commands.waitUntil(superstructure.atSuperstructureSetpoint
                                                .and(superstructure.desiredGoalNotStow)),
                                        intake.scoreCoral()
                                ),
                                superstructure.toSuperstructureGoal(
                                        ScoreCommands.Level.LevelMap.get(branch.level())
                                )
                        ),
                        Commands.waitUntil(superstructure.atSuperstructureSetpoint)
                                .withTimeout(2)
                )
        );
    }

    private Command intakeCoralFromHP() {
        return Commands.parallel(
                        superstructure.toSuperstructureGoal(Superstructure.Goal.HP),
                        intake.intakeCoralHP().asProxy(),
                        swerve.runWheelXCommand()
                ).until(gamepieceState.hasCoral);
    }

    private Command driveToCenterHP() {
        return swerve.driveToPose(() -> {
            final Pose2d currentPose = swerve.getPose();
            return currentPose.nearest(
                    FieldConstants.getHPPickupPoses()
            );
        });
    }

    private Command runStartingTrajectory(final AutoTrajectory startingTrajectory) {
        return Commands.sequence(
                Commands.runOnce(() -> intake.setCANRangeDistance(Units.inchesToMeters(6))),
                gamepieceState.setCoralState(GamepieceState.State.HOLDING),
                Commands.runOnce(reefState::reset),
                startingTrajectory.resetOdometry(),
                startingTrajectory.cmd()
        );
    }

    public AutoRoutine doNothing() {
        final AutoRoutine routine = autoFactory.newRoutine("DoNothing");

        routine.active().whileTrue(
                Commands.waitUntil(RobotModeTriggers.autonomous().negate())
        );

        return routine;
    }

    @SuppressWarnings("DuplicatedCode")
    public AutoRoutine cage0ToReef4() {
        final AutoRoutine routine = autoFactory.newRoutine("Cage0ToReef4");
        final AutoTrajectory cage0Reef4 = routine.trajectory("Cage0Reef4");
        final AutoTrajectory reef4ToRightHP = routine.trajectory("Reef4ToRightHP");
        final AutoTrajectory rightHPToReef4 = routine.trajectory("RightHPToReef4");
        final AutoTrajectory reef4ToRightHPForReef5 = routine.trajectory("Reef4ToRightHP");
        final AutoTrajectory rightHPToReef5Right = routine.trajectory("RightHPToReef5");
        final AutoTrajectory rightHPToReef5Left = routine.trajectory("RightHPToReef5");
        final AutoTrajectory reef5ToRightHP = routine.trajectory("Reef5ToRightHP");

        routine.active().onTrue(runStartingTrajectory(cage0Reef4));

        cage0Reef4.done().onTrue(
                Commands.sequence(
                        scoreAtLevel(new ReefState.Branch(Reef.Face.FOUR, Reef.Side.RIGHT, Reef.Level.L4))
                                .onlyIf(gamepieceState.hasCoral),
                        reef4ToRightHP.cmd()
                )
        );

        reef4ToRightHP.done().onTrue(
                Commands.sequence(
                        intakeCoralFromHP(),
                        rightHPToReef4.cmd()
                )
        );

        rightHPToReef4.done().onTrue(
                Commands.sequence(
                        scoreAtLevel(new ReefState.Branch(Reef.Face.FOUR, Reef.Side.LEFT, Reef.Level.L4))
                                .onlyIf(gamepieceState.hasCoral),
                        reef4ToRightHPForReef5.cmd()
                )
        );

        reef4ToRightHPForReef5.done().onTrue(
                Commands.sequence(
                        intakeCoralFromHP(),
                        rightHPToReef5Right.cmd()
                )
        );

        rightHPToReef5Right.done().onTrue(
                Commands.sequence(
                        scoreAtLevel(new ReefState.Branch(Reef.Face.FIVE, Reef.Side.RIGHT, Reef.Level.L4))
                                .onlyIf(gamepieceState.hasCoral),
                        reef5ToRightHP.cmd()
                )
        );

        reef5ToRightHP.done().onTrue(
                Commands.sequence(
                        intakeCoralFromHP(),
                        rightHPToReef5Left.cmd()
                )
        );

        rightHPToReef5Left.done().onTrue(
                Commands.sequence(
                        scoreAtLevel(new ReefState.Branch(Reef.Face.FIVE, Reef.Side.LEFT, Reef.Level.L4))
                                .onlyIf(gamepieceState.hasCoral),
                        swerve.runWheelXCommand()
                )
        );

        return routine;
    }

    @SuppressWarnings("DuplicatedCode")
    public AutoRoutine twoPieceCage0ToReef5() {
        final AutoRoutine routine = autoFactory.newRoutine("twoPieceCage0ToReef5");
        final AutoTrajectory cage0Reef5 = routine.trajectory("Cage0Reef5");
        final AutoTrajectory reef5ToRightHP = routine.trajectory("Reef5ToRightHP");
        final AutoTrajectory rightHPToReef5Left = routine.trajectory("RightHPToReef5");

        routine.active().onTrue(runStartingTrajectory(cage0Reef5));

        cage0Reef5.done().onTrue(
                Commands.sequence(
                        scoreAtLevel(new ReefState.Branch(Reef.Face.FIVE, Reef.Side.RIGHT, Reef.Level.L4))
                                .onlyIf(gamepieceState.hasCoral),
                        reef5ToRightHP.cmd()
                )
        );

        reef5ToRightHP.done().onTrue(
                Commands.sequence(
                        intakeCoralFromHP(),
                        rightHPToReef5Left.cmd()
                )
        );

        rightHPToReef5Left.done().onTrue(
                Commands.sequence(
                        scoreAtLevel(new ReefState.Branch(Reef.Face.FIVE, Reef.Side.LEFT, Reef.Level.L4))
                                .onlyIf(gamepieceState.hasCoral),
                        swerve.runWheelXCommand()
                )
        );

        return routine;
    }

    @SuppressWarnings("DuplicatedCode")
    public AutoRoutine twoPieceCage5ToReef1() {
        final AutoRoutine routine = autoFactory.newRoutine("twoPieceCage5ToReef1");
        final AutoTrajectory cage5Reef1 = routine.trajectory("Cage5Reef1");
        final AutoTrajectory reef1ToLeftHP = routine.trajectory("Reef1ToLeftHP");
        final AutoTrajectory leftHPToReef1Left = routine.trajectory("LeftHPToReef1");

        routine.active().onTrue(runStartingTrajectory(cage5Reef1));

        cage5Reef1.done().onTrue(
                Commands.sequence(
                        scoreAtLevel(new ReefState.Branch(Reef.Face.ONE, Reef.Side.LEFT, Reef.Level.L4))
                                .onlyIf(gamepieceState.hasCoral),
                        reef1ToLeftHP.cmd()
                )
        );

        reef1ToLeftHP.done().onTrue(
                Commands.sequence(
                        intakeCoralFromHP(),
                        leftHPToReef1Left.cmd()
                )
        );

        leftHPToReef1Left.done().onTrue(
                Commands.sequence(
                        scoreAtLevel(new ReefState.Branch(Reef.Face.ONE, Reef.Side.RIGHT, Reef.Level.L4))
                                .onlyIf(gamepieceState.hasCoral),
                        swerve.runWheelXCommand()
                )
        );

        return routine;
    }
}
