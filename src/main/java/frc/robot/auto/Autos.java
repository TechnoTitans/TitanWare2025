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
import frc.robot.utils.Container;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.function.Supplier;

public class Autos {
    public static final String LogKey = "Auto";

    private static final double AllowableDistanceFromHPForEarlyAlign = 0.75;

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

    private Command readyScoreAtLevel(final ReefState.Branch branch) {
        final Supplier<Pose2d> scoringPoseSupplier = () -> {
            final Pose2d scoringPose = FieldConstants.getBranchScoringPositions()
                    .get(branch.face())
                    .get(branch.side())
                    .get(branch.level());

            return scoreCommands
                    .offsetScoringPoseWithCoralPosition(scoringPose)
                    .transformBy(FieldConstants.AUTO_SCORING_DISTANCE_OFFSET);
        };

        final Superstructure.Goal goal = ScoreCommands.Level.LevelMap.get(branch.level());

        final Trigger atCloseReef = swerve.atPoseTrigger(
                scoringPoseSupplier,
                new HolonomicDriveController.PositionTolerance(
                        0.3,
                        Rotation2d.fromDegrees(8)
                ),
                new HolonomicDriveController.VelocityTolerance(
                        0.38,
                        Math.PI
                )
        );

        return Commands.sequence(
                superstructure.toInstantGoal(Superstructure.Goal.ALIGN_L4),
                Commands.waitUntil(atCloseReef),
                superstructure.runGoal(goal)
        );
    }

    private Command scoreAtLevel(final ReefState.Branch branch) {
        final Supplier<Pose2d> scoringPoseSupplier = () -> {
            final Pose2d scoringPose = FieldConstants.getBranchScoringPositions()
                    .get(branch.face())
                    .get(branch.side())
                    .get(branch.level());

            return scoreCommands
                    .offsetScoringPoseWithCoralPosition(scoringPose)
                    .transformBy(FieldConstants.AUTO_SCORING_DISTANCE_OFFSET);
        };

        final Superstructure.Goal goal = ScoreCommands.Level.LevelMap.get(branch.level());
        final Trigger atReef = swerve.atPoseTrigger(
                scoringPoseSupplier,
                new HolonomicDriveController.PositionTolerance(
                        0.05,
                        Rotation2d.fromDegrees(4)
                ),
                new HolonomicDriveController.VelocityTolerance(
                        0.13,
                        Math.PI / 3
                )
        );

        final Trigger atCloseReef = swerve.atPoseTrigger(
                scoringPoseSupplier,
                new HolonomicDriveController.PositionTolerance(
                        0.3,
                        Rotation2d.fromDegrees(8)
                ),
                new HolonomicDriveController.VelocityTolerance(
                        0.38,
                        Math.PI
                )
        );

        final Container<Boolean> wasEverAtReef = Container.of(false);

        return Commands.sequence(
                wasEverAtReef.set(false),
                Commands.deadline(
                        Commands.sequence(
                                Commands.waitUntil(atCloseReef),
                                Commands.deadline(
                                        Commands.sequence(
                                                Commands.waitUntil(superstructure.atSetpoint(goal))
                                                        .withTimeout(1.9),
                                                Commands.waitUntil(atReef.or(wasEverAtReef::get)),
                                                intake.scoreCoral(() -> Intake.ScoreMode.RUN_UNTIL_NO_CORAL).asProxy()
                                        ),
                                        superstructure.toGoal(goal)
                                )
                        ),
                        Commands.sequence(
                                swerve.runToPose(scoringPoseSupplier)
                                        .until(atReef),
                                wasEverAtReef.set(true),
                                swerve.runWheelXCommand()
                        )
                )
        );
    }

    private Command readyIntakeCoralFromHP() {
        return Commands.parallel(
                superstructure.runGoal(Superstructure.Goal.HP),
                intake.intakeCoralHP().asProxy()
        );
    }

    private Command driveIntoCoralStation(final ScoreCommands.CoralStation coralStation) {
        return Commands.parallel(
                superstructure.toGoal(Superstructure.Goal.HP),
                intake.intakeCoralHP().asProxy(),
                swerve.robotRelativeFacingAngle(
                        () -> 0.7,
                        () -> 0,
                        () -> ScoreCommands.CoralStation.getCoralStation(coralStation).getRotation()
                )
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

    private Command descoreUpperAlgae() {
        final Supplier<Pose2d> descorePoseSupplier = () -> swerve.getPose().
                nearest(new ArrayList<>(FieldConstants.getReefCenterPoses().values()))
                .plus(FieldConstants.ALGAE_DESCORE_DISTANCE_OFFSET);

        final Supplier<Pose2d> safeReefPoseSupplier = () -> descorePoseSupplier.get()
                .transformBy(FieldConstants.ALGAE_SAFE_REEF_OFFSET);

        final Trigger safeFromReef = swerve.atPoseTrigger(
                safeReefPoseSupplier,
                new HolonomicDriveController.PositionTolerance(
                        0.1,
                        Rotation2d.fromDegrees(8)
                )
        );

        return Commands.deadline(
                Commands.sequence(
                        swerve.runToPose(descorePoseSupplier)
                                .until(gamepieceState.hasAlgae),
                        Commands.waitSeconds(0.2),
                        swerve.runToPose(safeReefPoseSupplier).until(safeFromReef)
                                .withTimeout(0.35)
                ),
                superstructure.toGoal(Superstructure.Goal.UPPER_ALGAE),
                intake.intakeAlgae().asProxy()
        ).withName("DescoreUpperAlgae");
    }

    private Command descoreLowerAlgae() {
        final Supplier<Pose2d> descorePoseSupplier = () -> swerve.getPose().
                nearest(new ArrayList<>(FieldConstants.getReefCenterPoses().values()))
                .plus(FieldConstants.ALGAE_DESCORE_DISTANCE_OFFSET);

        final Supplier<Pose2d> safeReefPoseSupplier = () -> descorePoseSupplier.get()
                .transformBy(FieldConstants.ALGAE_SAFE_REEF_OFFSET);

        final Trigger safeFromReef = swerve.atPoseTrigger(
                safeReefPoseSupplier,
                new HolonomicDriveController.PositionTolerance(
                        0.1,
                        Rotation2d.fromDegrees(8)
                )
        );

        return Commands.deadline(
                Commands.sequence(
                        swerve.runToPose(descorePoseSupplier)
                                .until(gamepieceState.hasAlgae),
                        Commands.waitSeconds(0.2),
                        swerve.runToPose(safeReefPoseSupplier).until(safeFromReef)
                                .withTimeout(0.35)
                ),
                superstructure.toGoal(Superstructure.Goal.LOWER_ALGAE),
                intake.intakeAlgae().asProxy()
        ).withName("DescoreLowerAlgae");
    }

    public AutoRoutine threePieceCage1() {
        final AutoRoutine routine = autoFactory.newRoutine("threePieceCage1");
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

        final ReefState.Branch fourLeftL4 = new ReefState.Branch(Reef.Face.FOUR, Reef.Side.LEFT, Reef.Level.AUTO_L4);
        startToReef.active().whileTrue(
                readyScoreAtLevel(fourLeftL4)
        );

        startToReef.done().onTrue(
                Commands.sequence(
                        scoreAtLevel(fourLeftL4)
                                .onlyIf(gamepieceState.hasCoral)
                                .asProxy(),
                        reef4ToRightHP.cmd()
                                .asProxy()
                )
        );

        reef4ToRightHP.active().whileTrue(readyIntakeCoralFromHP());

        final ReefState.Branch fiveRightL4 = new ReefState.Branch(Reef.Face.FIVE, Reef.Side.RIGHT, Reef.Level.AUTO_L4);
        reef4ToRightHP.done().onTrue(
                Commands.sequence(
                        driveIntoCoralStation(ScoreCommands.CoralStation.RIGHT),
                        Commands.parallel(
                                firstRightHPToReef5.cmd(),
                                Commands.sequence(
                                        Commands.waitUntil(farEnoughAwayFromHP)
                                                .withTimeout(0.3),
                                        readyScoreAtLevel(fiveRightL4)
                                )
                        )
                )
        );

        firstRightHPToReef5.done().onTrue(
                Commands.sequence(
                        scoreAtLevel(fiveRightL4)
                                .onlyIf(gamepieceState.hasCoral)
                                .asProxy(),
                        reef5ToRightHP.cmd()
                                .asProxy()
                )
        );

        reef5ToRightHP.active().whileTrue(readyIntakeCoralFromHP());

        final ReefState.Branch fiveLeftL4 = new ReefState.Branch(Reef.Face.FIVE, Reef.Side.LEFT, Reef.Level.AUTO_L4);
        reef5ToRightHP.done().onTrue(
                Commands.sequence(
                        driveIntoCoralStation(ScoreCommands.CoralStation.RIGHT),
                        Commands.parallel(
                                secondRightHPToReef5.cmd(),
                                Commands.sequence(
                                        Commands.waitUntil(farEnoughAwayFromHP)
                                                .withTimeout(0.3),
                                        readyScoreAtLevel(fiveLeftL4)
                                )
                        )
                )
        );

        secondRightHPToReef5.done().onTrue(
                Commands.sequence(
                        scoreAtLevel(fiveLeftL4)
                                .onlyIf(gamepieceState.hasCoral)
                                .asProxy(),
                        moveEndOfAuto.cmd()
                                .asProxy()
                )
        );

        moveEndOfAuto.done().onTrue(
                swerve.runWheelXCommand()
        );

        return routine;
    }

    public AutoRoutine threePieceCage4() {
        final AutoRoutine routine = autoFactory.newRoutine("threePieceCage4");
        final AutoTrajectory startToReef = routine.trajectory("Cage4Reef2");
        final AutoTrajectory reef2ToLeftHP = routine.trajectory("Reef2ToLeftHP");
        final AutoTrajectory leftHPToReef1Right = routine.trajectory("LeftHPToReef1Right");
        final AutoTrajectory reef1ToLeftHP = routine.trajectory("Reef1ToLeftHP");
        final AutoTrajectory leftHPToReef1Left = routine.trajectory("LeftHPToReef1Left");
        final AutoTrajectory moveEndOfAuto = routine.trajectory("Reef1ToLeftHP");

        final Trigger farEnoughAwayFromHP = new Trigger(() -> {
            final Pose2d pose = swerve.getPose();
            final Pose2d closestHPPose = pose.nearest(FieldConstants.getHPPickupPoses());

            return pose.getTranslation()
                    .getDistance(closestHPPose.getTranslation()) >= AllowableDistanceFromHPForEarlyAlign;
        });

        routine.active().onTrue(runStartingTrajectory(startToReef));

        final ReefState.Branch twoRightL4 = new ReefState.Branch(Reef.Face.TWO, Reef.Side.RIGHT, Reef.Level.AUTO_L4);
        startToReef.active().whileTrue(
                readyScoreAtLevel(twoRightL4)
        );

        startToReef.done().onTrue(
                Commands.sequence(
                        scoreAtLevel(twoRightL4)
                                .onlyIf(gamepieceState.hasCoral)
                                .asProxy(),
                        reef2ToLeftHP.cmd()
                                .asProxy()
                )
        );

        reef2ToLeftHP.active().whileTrue(readyIntakeCoralFromHP());

        final ReefState.Branch oneRightL4 = new ReefState.Branch(Reef.Face.ONE, Reef.Side.RIGHT, Reef.Level.AUTO_L4);
        reef2ToLeftHP.done().onTrue(
                Commands.sequence(
                        driveIntoCoralStation(ScoreCommands.CoralStation.LEFT),
                        Commands.parallel(
                                leftHPToReef1Right.cmd(),
                                Commands.sequence(
                                        Commands.waitUntil(farEnoughAwayFromHP)
                                                .withTimeout(0.3),
                                        readyScoreAtLevel(oneRightL4)
                                )
                        )
                )
        );

        leftHPToReef1Right.done().onTrue(
                Commands.sequence(
                        scoreAtLevel(oneRightL4)
                                .onlyIf(gamepieceState.hasCoral)
                                .asProxy(),
                        reef1ToLeftHP.cmd()
                                .asProxy()
                )
        );

        reef1ToLeftHP.active().whileTrue(readyIntakeCoralFromHP());

        final ReefState.Branch oneLeftL4 = new ReefState.Branch(Reef.Face.ONE, Reef.Side.LEFT, Reef.Level.AUTO_L4);
        reef1ToLeftHP.done().onTrue(
                Commands.sequence(
                        driveIntoCoralStation(ScoreCommands.CoralStation.LEFT),
                        Commands.parallel(
                                leftHPToReef1Left.cmd(),
                                Commands.sequence(
                                        Commands.waitUntil(farEnoughAwayFromHP)
                                                .withTimeout(0.3),
                                        readyScoreAtLevel(oneLeftL4)
                                )
                        )
                )
        );

        leftHPToReef1Left.done().onTrue(
                Commands.sequence(
                        scoreAtLevel(oneLeftL4)
                                .onlyIf(gamepieceState.hasCoral)
                                .asProxy(),
                        moveEndOfAuto.cmd()
                                .asProxy()
                )
        );

        moveEndOfAuto.done().onTrue(
                swerve.runWheelXCommand()
        );

        return routine;
    }

    public AutoRoutine centerBarge() {
        final AutoRoutine routine = autoFactory.newRoutine("centerBarge");
        final AutoTrajectory startToReef = routine.trajectory("CenterReef3");
        final AutoTrajectory reef3ToBarge = routine.trajectory("Reef3ToBarge");
        final AutoTrajectory bargeToTaxi = routine.trajectory("BargeToTaxi");

        routine.active().onTrue(runStartingTrajectory(startToReef));

        final ReefState.Branch threeRightL4 = new ReefState.Branch(
                Reef.Face.THREE_FACING_CLIMB, Reef.Side.RIGHT, Reef.Level.AUTO_L4
        );
        startToReef.active().whileTrue(
                readyScoreAtLevel(threeRightL4)
        );

        startToReef.done().onTrue(
                Commands.sequence(
                        scoreAtLevel(threeRightL4)
                                .onlyIf(gamepieceState.hasCoral),
                        descoreLowerAlgae(),
                        Commands.waitUntil(superstructure.atSetpoint(Superstructure.Goal.STOW)),
                        reef3ToBarge.cmd()
                )
        );

        reef3ToBarge.done().onTrue(
                Commands.sequence(
                        scoreCommands.scoreBarge(),
                        bargeToTaxi.cmd()
                )
        );

        bargeToTaxi.done().onTrue(
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

        reef4ToRightHP.active().onTrue(readyIntakeCoralFromHP());

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

        reef4ToRightHP.active().onTrue(readyIntakeCoralFromHP());

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

        reef4ToRightHP.active().onTrue(readyIntakeCoralFromHP());

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

        reef2ToLeftHP.active().onTrue(readyIntakeCoralFromHP());

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

        reef2ToLeftHP.active().onTrue(readyIntakeCoralFromHP());

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

        reef2ToLeftHP.active().onTrue(readyIntakeCoralFromHP());

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
}
