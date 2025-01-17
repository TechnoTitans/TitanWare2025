package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.state.GamePieceState;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.superstructure.Superstructure;

import java.util.function.DoubleSupplier;

public class ScoreCommands {
    public enum Side {
        NONE(0),
        LEFT(Units.inchesToMeters(-6)),
        RIGHT(Units.inchesToMeters(6));

        public final double xOffsetFromCenterOfSide;
        Side(final double xOffsetFromCenterOfSide) {
            this.xOffsetFromCenterOfSide = xOffsetFromCenterOfSide;
        }
    }

    public enum Level {
        L1(Superstructure.SuperstructureGoal.L1),
        L2(Superstructure.SuperstructureGoal.L2),
        L3(Superstructure.SuperstructureGoal.L3),
        L4(Superstructure.SuperstructureGoal.L4);

        public final Superstructure.SuperstructureGoal superstructureGoal;
        Level(final Superstructure.SuperstructureGoal superstructureGoal) {
            this.superstructureGoal = superstructureGoal;
        }
    }

    public record ScorePosition(
            Side side,
            Level level
    ) {}

    private final Swerve swerve;
    private final Superstructure superstructure;
    private final Intake intake;
    private final GamePieceState gamePieceState;

    public ScoreCommands(
            final Swerve swerve,
            final Superstructure superstructure,
            final Intake intake,
            final GamePieceState gamePieceState
    ) {
        this.swerve = swerve;
        this.superstructure = superstructure;
        this.intake = intake;
        this.gamePieceState = gamePieceState;
    }

    public ScorePosition getScoreLevel(
            final double rightXStickInput,
            final double rightYStickInput
    ) {
        final double rightXStickInputWithDeadband = MathUtil.applyDeadband(rightXStickInput, 0.1);
        final double rightYStickInputWithDeadband = MathUtil.applyDeadband(rightYStickInput, 0.1);

        final Side side;
        final Level level;

        if (rightXStickInputWithDeadband > 0) {
            side = Side.RIGHT;
        } else if (rightXStickInputWithDeadband < 0) {
            side = Side.LEFT;
        } else {
            side = Side.NONE;
        }

        if (rightYStickInputWithDeadband > 0.5) {
            level = Level.L4;
        } else if (rightYStickInputWithDeadband > 0) {
            level = Level.L3;
        } else if (rightYStickInputWithDeadband <= -0.5) {
            level = Level.L1;
        } else if (rightYStickInputWithDeadband < 0) {
            level = Level.L2;
        } else {
            level = Level.L1;
        }

        return new ScorePosition(side, level);
    }

    public Command scoreAtLevel(
            final ScorePosition scorePosition
    ) {
        return Commands.parallel(
                Commands.deadline(
                        superstructure.toSuperstructureGoal(scorePosition.level.superstructureGoal),
                        Commands.sequence(
                                Commands.waitUntil(superstructure.atSuperstructureSetpoint),
                                intake.runCoralRollerVelocity(-4)
                                        .until(intake.isCoralPresent.negate())
                        )
//                        swerve.driveToPose(
//                                swerve.getPose().transformBy(
//                                        swerve.getPose().getRotation().toTransform().transformBy(
//                                                new Transform2d(
//                                                        scorePosition.side.xOffsetFromCenterOfSide,
//                                                        0
//                                                )
//                                        )
//                                )
//                        )
                )
        );
    }
}
