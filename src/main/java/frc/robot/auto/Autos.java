package frc.robot.auto;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.ScoreCommands;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.FieldConstants.Reef;
import frc.robot.state.GamepieceState;
import frc.robot.state.ReefState;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.drive.controllers.HolonomicDriveController;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.vision.PhotonVision;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

public class Autos {
    // choreo-cli.exe --chor choreo.chor --all-trajectory --generate
    public static final String LogKey = "Auto";

    private static final double AllowableDistanceFromHPForEarlyAlign = 0.2;

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
        final Supplier<Pose2d> scoringPoseSupplier = () -> {
            final Pose2d scoringPose = FieldConstants.getBranchScoringPositions()
                    .get(branch.face())
                    .get(branch.side())
                    .get(branch.level());

            return scoreCommands.offsetScoringPoseWithCoralPosition(scoringPose);
        };

        final Superstructure.Goal goal = ScoreCommands.Level.LevelMap.get(branch.level());
        final Trigger atReef = swerve.atPoseTrigger(
                scoringPoseSupplier,
                new HolonomicDriveController.Tolerance(
                        0.2,
                        0.35,
                        Rotation2d.fromDegrees(8),
                        Math.PI / 4
                )
        );

        return Commands.deadline(
                Commands.sequence(
                        Commands.waitUntil(atReef),
                        Commands.deadline(
                                Commands.sequence(
                                        Commands.waitUntil(superstructure.atSetpoint(goal))
                                                .withTimeout(2),
                                        intake.scoreCoral()
                                ),
                                superstructure.toGoal(goal)
                        ),
                        Commands.waitUntil(superstructure.extendedBeyond(0.9).negate())
                                .withTimeout(0.2)
                ),
                swerve.driveToPose(scoringPoseSupplier)
                        .andThen(swerve.runWheelXCommand())
        );
    }

    private Command intakeCoralFromHP(final ScoreCommands.CoralStation coralStation) {
        return Commands.parallel(
                superstructure.toGoal(Superstructure.Goal.HP),
                intake.intakeCoralHP().asProxy(),
                swerve.runToPose(() -> ScoreCommands.CoralStation.getCoralStation(coralStation))
        ).until(gamepieceState.hasCoral);
    }

    private Command runStartingTrajectory(final AutoTrajectory startingTrajectory) {
        return Commands.sequence(
                Commands.runOnce(() -> intake.setTOFDistance(Units.inchesToMeters(6))),
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

    public AutoRoutine twoPieceCage3ToReef2And1() {
        final AutoRoutine routine = autoFactory.newRoutine("twoPieceCage3ToReef2And1");
        final AutoTrajectory cage4Reef2 = routine.trajectory("Cage3Reef2");
        final AutoTrajectory reef2ToLeftHP = routine.trajectory("Reef2ToLeftHP");
        final AutoTrajectory leftHPToReef1Left = routine.trajectory("LeftHPToReef1");
        final AutoTrajectory moveEndOfAuto = routine.trajectory("Reef2ToLeftHP");

        routine.active().onTrue(runStartingTrajectory(cage4Reef2));

        cage4Reef2.active().whileTrue(
                superstructure.runGoal(Superstructure.Goal.ALIGN_L4)
        );

        cage4Reef2.done().onTrue(
                Commands.sequence(
                        scoreAtLevel(new ReefState.Branch(Reef.Face.TWO, Reef.Side.RIGHT, Reef.Level.L4))
                                .onlyIf(gamepieceState.hasCoral),
                        reef2ToLeftHP.cmd()
                )
        );

        reef2ToLeftHP.done().onTrue(
                Commands.sequence(
                        intakeCoralFromHP(ScoreCommands.CoralStation.LEFT),
                        leftHPToReef1Left.cmd()
                                .alongWith(superstructure.toInstantGoal(Superstructure.Goal.ALIGN_L4))
                )
        );

        leftHPToReef1Left.done().onTrue(
                Commands.sequence(
                        scoreAtLevel(new ReefState.Branch(Reef.Face.ONE, Reef.Side.LEFT, Reef.Level.L4))
                                .onlyIf(gamepieceState.hasCoral),
                        moveEndOfAuto.cmd()
                )
        );

        moveEndOfAuto.done().onTrue(
                swerve.runWheelXCommand()
        );

        return routine;
    }

    public AutoRoutine twoPieceCage2ToReef4And5() {
        final AutoRoutine routine = autoFactory.newRoutine("twoPieceCage2ToReef4And5");
        final AutoTrajectory cage2Reef4 = routine.trajectory("Cage2Reef4");
        final AutoTrajectory reef4ToRightHP = routine.trajectory("Reef4ToRightHP");
        final AutoTrajectory rightHPToReef5 = routine.trajectory("RightHPToReef5");
        final AutoTrajectory moveEndOfAuto = routine.trajectory("Reef4ToRightHP");

        routine.active().onTrue(runStartingTrajectory(cage2Reef4));

        cage2Reef4.active().whileTrue(
                superstructure.runGoal(Superstructure.Goal.ALIGN_L4)
        );

        cage2Reef4.done().onTrue(
                Commands.sequence(
                        scoreAtLevel(new ReefState.Branch(Reef.Face.FOUR, Reef.Side.LEFT, Reef.Level.L4))
                                .onlyIf(gamepieceState.hasCoral),
                        reef4ToRightHP.cmd()
                )
        );

        reef4ToRightHP.done().onTrue(
                Commands.sequence(
                        intakeCoralFromHP(ScoreCommands.CoralStation.RIGHT),
                        rightHPToReef5.cmd()
                                .alongWith(superstructure.toInstantGoal(Superstructure.Goal.ALIGN_L4))
                )
        );

        rightHPToReef5.done().onTrue(
                Commands.sequence(
                        scoreAtLevel(new ReefState.Branch(Reef.Face.FIVE, Reef.Side.RIGHT, Reef.Level.L4))
                                .onlyIf(gamepieceState.hasCoral),
                        moveEndOfAuto.cmd()
                )
        );

        moveEndOfAuto.done().onTrue(
                swerve.runWheelXCommand()
        );

        return routine;
    }

    public AutoRoutine threePieceCage2ToReef4And5() {
        final AutoRoutine routine = autoFactory.newRoutine("threePieceCage2ToReef4And5");
        final AutoTrajectory startToReef = routine.trajectory("Cage2Reef4");
        final AutoTrajectory reef4ToRightHP = routine.trajectory("Reef4ToRightHP");
        final AutoTrajectory firstRightHPToReef5 = routine.trajectory("RightHPToReef5");
        final AutoTrajectory reef5ToRightHP = routine.trajectory("Reef5ToRightHP");
        final AutoTrajectory secondRightHPToReef5 = routine.trajectory("RightHPToReef5");
        final AutoTrajectory moveEndOfAuto = routine.trajectory("Reef5ToRightHP");

        routine.active().onTrue(runStartingTrajectory(startToReef));

        startToReef.active().whileTrue(
                superstructure.runGoal(Superstructure.Goal.ALIGN_L4)
        );

        startToReef.done().onTrue(
                Commands.sequence(
                        scoreAtLevel(new ReefState.Branch(Reef.Face.FOUR, Reef.Side.LEFT, Reef.Level.L4))
                                .onlyIf(gamepieceState.hasCoral),
                        reef4ToRightHP.cmd()
                )
        );

        reef4ToRightHP.done().onTrue(
                Commands.sequence(
                        intakeCoralFromHP(ScoreCommands.CoralStation.RIGHT),
                        firstRightHPToReef5.cmd()
                                .alongWith(superstructure.toInstantGoal(Superstructure.Goal.ALIGN_L4))
                )
        );

        firstRightHPToReef5.done().onTrue(
                Commands.sequence(
                        scoreAtLevel(new ReefState.Branch(Reef.Face.FIVE, Reef.Side.RIGHT, Reef.Level.L4))
                                .onlyIf(gamepieceState.hasCoral),
                        reef5ToRightHP.cmd()
                )
        );

        reef5ToRightHP.done().onTrue(
                Commands.sequence(
                        intakeCoralFromHP(ScoreCommands.CoralStation.RIGHT),
                        secondRightHPToReef5.cmd()
                                .alongWith(superstructure.toInstantGoal(Superstructure.Goal.ALIGN_L4))
                )
        );

        secondRightHPToReef5.done().onTrue(
                Commands.sequence(
                        scoreAtLevel(new ReefState.Branch(Reef.Face.FIVE, Reef.Side.LEFT, Reef.Level.L4))
                                .onlyIf(gamepieceState.hasCoral),
                        moveEndOfAuto.cmd()
                )
        );

        moveEndOfAuto.done().onTrue(
                swerve.runWheelXCommand()
        );

        return routine;
    }

    public AutoRoutine threePieceCage3ToReef2And1() {
        final AutoRoutine routine = autoFactory.newRoutine("threePieceCage3ToReef2And1");
        final AutoTrajectory startToReef = routine.trajectory("Cage3Reef2");
        final AutoTrajectory reef4ToRightHP = routine.trajectory("Reef2ToLeftHP");
        final AutoTrajectory firstRightHPToReef5 = routine.trajectory("LeftHPToReef1");
        final AutoTrajectory reef5ToRightHP = routine.trajectory("Reef1ToLeftHP");
        final AutoTrajectory secondRightHPToReef5 = routine.trajectory("LeftHPToReef1");
        final AutoTrajectory moveEndOfAuto = routine.trajectory("Reef1ToLeftHP");

        final Trigger farEnoughAwayFromHP = new Trigger(() -> {
            final Pose2d pose = swerve.getPose();
            final Pose2d closestHPPose = pose.nearest(FieldConstants.getHPPickupPoses());

            return pose.getTranslation()
                    .getDistance(closestHPPose.getTranslation()) >= AllowableDistanceFromHPForEarlyAlign;
        });

        routine.active().onTrue(runStartingTrajectory(startToReef));

        startToReef.active().whileTrue(
                superstructure.runGoal(Superstructure.Goal.ALIGN_L4)
        );

        startToReef.done().onTrue(
                Commands.sequence(
                        scoreAtLevel(new ReefState.Branch(Reef.Face.TWO, Reef.Side.RIGHT, Reef.Level.L4))
                                .onlyIf(gamepieceState.hasCoral),
                        reef4ToRightHP.cmd()
                )
        );

        reef4ToRightHP.done().onTrue(
                Commands.sequence(
                        intakeCoralFromHP(ScoreCommands.CoralStation.LEFT),
                        Commands.parallel(
                                firstRightHPToReef5.cmd(),
                                Commands.sequence(
                                        Commands.waitUntil(farEnoughAwayFromHP)
                                                .withTimeout(0.3),
                                        superstructure.toInstantGoal(Superstructure.Goal.ALIGN_L4)
                                )
                        )
                )
        );

        firstRightHPToReef5.done().onTrue(
                Commands.sequence(
                        scoreAtLevel(new ReefState.Branch(Reef.Face.ONE, Reef.Side.LEFT, Reef.Level.L4))
                                .onlyIf(gamepieceState.hasCoral),
                        reef5ToRightHP.cmd()
                )
        );

        reef5ToRightHP.done().onTrue(
                Commands.sequence(
                        intakeCoralFromHP(ScoreCommands.CoralStation.LEFT),
                        Commands.parallel(
                                secondRightHPToReef5.cmd(),
                                Commands.sequence(
                                        Commands.waitUntil(farEnoughAwayFromHP)
                                                .withTimeout(0.3),
                                        superstructure.toInstantGoal(Superstructure.Goal.ALIGN_L4)
                                )
                        )
                )
        );

        secondRightHPToReef5.done().onTrue(
                Commands.sequence(
                        scoreAtLevel(new ReefState.Branch(Reef.Face.ONE, Reef.Side.RIGHT, Reef.Level.L4))
                                .onlyIf(gamepieceState.hasCoral),
                        moveEndOfAuto.cmd()
                )
        );

        moveEndOfAuto.done().onTrue(
                swerve.runWheelXCommand()
        );

        return routine;
    }
}
