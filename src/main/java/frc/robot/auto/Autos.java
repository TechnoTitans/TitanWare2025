package frc.robot.auto;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
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
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.vision.PhotonVision;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

public class Autos {
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
        final Trigger atReef = swerve.atPoseAndStoppedTrigger(scoringPoseSupplier);

        return Commands.deadline(
                Commands.sequence(
                        Commands.waitUntil(atReef),
                        Commands.deadline(
                                Commands.sequence(
                                        Commands.waitUntil(superstructure.atSetpoint(goal))
                                                .withTimeout(1.9),
                                        intake.scoreCoral()
                                ),
                                superstructure.toGoal(goal)
                        )
                ),
                swerve.runToPose(scoringPoseSupplier)
                        .until(atReef)
                        .andThen(swerve.runWheelXCommand())
        );
    }

    private Command driveIntoCoralStation(final ScoreCommands.CoralStation coralStation) {
        return swerve.robotRelativeFacingAngle(
                () -> 1,
                () -> 0,
                () -> ScoreCommands.CoralStation.getCoralStation(coralStation).getRotation()
        ).until(gamepieceState.hasCoral);
    }

    private Command intakeCoralFromHP() {
        return Commands.parallel(
                superstructure.toGoal(Superstructure.Goal.HP),
                intake.intakeCoralHP().asProxy()
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

    public AutoRoutine straight() {
        final AutoRoutine routine = autoFactory.newRoutine("straight");
        final AutoTrajectory cage0Reef4 = routine.trajectory("testMeter");

        routine.active().onTrue(
                Commands.sequence(
                        cage0Reef4.resetOdometry(),
                        cage0Reef4.cmd()
                )
        );

        cage0Reef4.done().onTrue(
                swerve.runWheelXCommand()
        );

        return routine;
    }

    public AutoRoutine twoPieceCage0() {
        final AutoRoutine routine = autoFactory.newRoutine("twoPieceCage0");
        final AutoTrajectory cage0Reef4 = routine.trajectory("Cage0Reef4");
        final AutoTrajectory reef4ToRightHP = routine.trajectory("Reef4ToRightHP");
        final AutoTrajectory rightHPToReef5 = routine.trajectory("RightHPToReef5");
        final AutoTrajectory moveEndOfAuto = routine.trajectory("Reef5ToRightHP");

        routine.active().onTrue(runStartingTrajectory(cage0Reef4));

        cage0Reef4.done().onTrue(
                Commands.sequence(
                        scoreAtLevel(new ReefState.Branch(Reef.Face.FOUR, Reef.Side.LEFT, Reef.Level.L4))
                                .onlyIf(gamepieceState.hasCoral),
                        reef4ToRightHP.cmd()
                )
        );

        reef4ToRightHP.active().onTrue(intakeCoralFromHP());

        reef4ToRightHP.done().onTrue(
                Commands.sequence(
                        driveIntoCoralStation(ScoreCommands.CoralStation.RIGHT),
                        rightHPToReef5.cmd()
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

    public AutoRoutine twoPieceCage1() {
        final AutoRoutine routine = autoFactory.newRoutine("twoPieceCage1");
        final AutoTrajectory cage0Reef4 = routine.trajectory("Cage1Reef4");
        final AutoTrajectory reef4ToRightHP = routine.trajectory("Reef4ToRightHP");
        final AutoTrajectory rightHPToReef5 = routine.trajectory("RightHPToReef5");
        final AutoTrajectory moveEndOfAuto = routine.trajectory("Reef5ToRightHP");

        routine.active().onTrue(runStartingTrajectory(cage0Reef4));

        cage0Reef4.done().onTrue(
                Commands.sequence(
                        scoreAtLevel(new ReefState.Branch(Reef.Face.FOUR, Reef.Side.LEFT, Reef.Level.L4))
                                .onlyIf(gamepieceState.hasCoral),
                        reef4ToRightHP.cmd()
                )
        );

        reef4ToRightHP.active().onTrue(intakeCoralFromHP());

        reef4ToRightHP.done().onTrue(
                Commands.sequence(
                        driveIntoCoralStation(ScoreCommands.CoralStation.RIGHT),
                        rightHPToReef5.cmd()
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

    public AutoRoutine twoPieceCage2() {
        final AutoRoutine routine = autoFactory.newRoutine("twoPieceCage2");
        final AutoTrajectory cage2Reef4 = routine.trajectory("Cage2Reef4");
        final AutoTrajectory reef4ToRightHP = routine.trajectory("Reef4ToRightHP");
        final AutoTrajectory rightHPToReef5 = routine.trajectory("RightHPToReef5");
        final AutoTrajectory moveEndOfAuto = routine.trajectory("Reef5ToRightHP");

        routine.active().onTrue(runStartingTrajectory(cage2Reef4));

        cage2Reef4.done().onTrue(
                Commands.sequence(
                        scoreAtLevel(new ReefState.Branch(Reef.Face.FOUR, Reef.Side.LEFT, Reef.Level.L4))
                                .onlyIf(gamepieceState.hasCoral),
                        reef4ToRightHP.cmd()
                )
        );

        reef4ToRightHP.active().onTrue(intakeCoralFromHP());

        reef4ToRightHP.done().onTrue(
                Commands.sequence(
                        driveIntoCoralStation(ScoreCommands.CoralStation.RIGHT),
                        rightHPToReef5.cmd()
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

    public AutoRoutine twoPieceCage3() {
        final AutoRoutine routine = autoFactory.newRoutine("twoPieceCage3");
        final AutoTrajectory cage4Reef2 = routine.trajectory("Cage3Reef2");
        final AutoTrajectory reef2ToLeftHP = routine.trajectory("Reef2ToLeftHP");
        final AutoTrajectory leftHPToReef1Left = routine.trajectory("LeftHPToReef1");
        final AutoTrajectory moveEndOfAuto = routine.trajectory("Reef1ToLeftHP");

        routine.active().onTrue(runStartingTrajectory(cage4Reef2));

        cage4Reef2.done().onTrue(
                Commands.sequence(
                        scoreAtLevel(new ReefState.Branch(Reef.Face.TWO, Reef.Side.RIGHT, Reef.Level.L4))
                                .onlyIf(gamepieceState.hasCoral),
                        reef2ToLeftHP.cmd()
                )
        );

        reef2ToLeftHP.active().onTrue(intakeCoralFromHP());

        reef2ToLeftHP.done().onTrue(
                Commands.sequence(
                        driveIntoCoralStation(ScoreCommands.CoralStation.LEFT),
                        leftHPToReef1Left.cmd()
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

    public AutoRoutine twoPieceCage4() {
        final AutoRoutine routine = autoFactory.newRoutine("twoPieceCage4");
        final AutoTrajectory cage0Reef2 = routine.trajectory("Cage4Reef2");
        final AutoTrajectory reef2ToLeftHP = routine.trajectory("Reef2ToLeftHP");
        final AutoTrajectory leftHPToReef1Left = routine.trajectory("LeftHPToReef1");
        final AutoTrajectory moveEndOfAuto = routine.trajectory("Reef1ToLeftHP");

        routine.active().onTrue(runStartingTrajectory(cage0Reef2));

        cage0Reef2.done().onTrue(
                Commands.sequence(
                        scoreAtLevel(new ReefState.Branch(Reef.Face.TWO, Reef.Side.RIGHT, Reef.Level.L4))
                                .onlyIf(gamepieceState.hasCoral),
                        reef2ToLeftHP.cmd()
                )
        );

        reef2ToLeftHP.active().onTrue(intakeCoralFromHP());

        reef2ToLeftHP.done().onTrue(
                Commands.sequence(
                        driveIntoCoralStation(ScoreCommands.CoralStation.LEFT),
                        leftHPToReef1Left.cmd()
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

    public AutoRoutine twoPieceCage5() {
        final AutoRoutine routine = autoFactory.newRoutine("twoPieceCage5");
        final AutoTrajectory cage0Reef2 = routine.trajectory("Cage5Reef2");
        final AutoTrajectory reef2ToLeftHP = routine.trajectory("Reef2ToLeftHP");
        final AutoTrajectory leftHPToReef1Left = routine.trajectory("LeftHPToReef1");
        final AutoTrajectory moveEndOfAuto = routine.trajectory("Reef1ToLeftHP");

        routine.active().onTrue(runStartingTrajectory(cage0Reef2));

        cage0Reef2.done().onTrue(
                Commands.sequence(
                        scoreAtLevel(new ReefState.Branch(Reef.Face.TWO, Reef.Side.RIGHT, Reef.Level.L4))
                                .onlyIf(gamepieceState.hasCoral),
                        reef2ToLeftHP.cmd()
                )
        );

        reef2ToLeftHP.active().onTrue(intakeCoralFromHP());

        reef2ToLeftHP.done().onTrue(
                Commands.sequence(
                        driveIntoCoralStation(ScoreCommands.CoralStation.LEFT),
                        leftHPToReef1Left.cmd()
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

    public AutoRoutine threePieceCage1ToReef4And5() {
        final AutoRoutine routine = autoFactory.newRoutine("threePieceCage1ToReef4And5");
        final AutoTrajectory startToReef = routine.trajectory("Cage1Reef4");
        final AutoTrajectory reef4ToRightHP = routine.trajectory("Reef4ToRightHP");
        final AutoTrajectory firstRightHPToReef5 = routine.trajectory("RightHPToReef5Right");
        final AutoTrajectory reef5ToRightHP = routine.trajectory("Reef5ToRightHP");
        final AutoTrajectory secondRightHPToReef5 = routine.trajectory("RightHPToReef5Left");
        final AutoTrajectory moveEndOfAuto = routine.trajectory("Reef5ToRightHP");

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
                        scoreAtLevel(new ReefState.Branch(Reef.Face.FOUR, Reef.Side.LEFT, Reef.Level.L4))
                                .onlyIf(gamepieceState.hasCoral),
                        reef4ToRightHP.cmd()
                )
        );

        reef4ToRightHP.active().onTrue(intakeCoralFromHP());

        reef4ToRightHP.done().onTrue(
                Commands.sequence(
                        driveIntoCoralStation(ScoreCommands.CoralStation.RIGHT),
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
                        scoreAtLevel(new ReefState.Branch(Reef.Face.FIVE, Reef.Side.RIGHT, Reef.Level.L4))
                                .onlyIf(gamepieceState.hasCoral),
                        reef5ToRightHP.cmd()
                )
        );

        reef5ToRightHP.active().onTrue(intakeCoralFromHP());

        reef5ToRightHP.done().onTrue(
                Commands.sequence(
                        driveIntoCoralStation(ScoreCommands.CoralStation.RIGHT),
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

    public AutoRoutine threePieceCage4ToReef2And1() {
        final AutoRoutine routine = autoFactory.newRoutine("threePieceCage4ToReef2And1");
        final AutoTrajectory startToReef = routine.trajectory("Cage4Reef2");
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

        reef4ToRightHP.active().onTrue(intakeCoralFromHP());

        reef4ToRightHP.done().onTrue(
                Commands.sequence(
                        driveIntoCoralStation(ScoreCommands.CoralStation.LEFT),
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

        reef5ToRightHP.active().onTrue(intakeCoralFromHP());

        reef5ToRightHP.done().onTrue(
                Commands.sequence(
                        driveIntoCoralStation(ScoreCommands.CoralStation.LEFT),
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
