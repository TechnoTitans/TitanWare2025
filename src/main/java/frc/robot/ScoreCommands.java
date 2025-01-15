package frc.robot;

import frc.robot.state.GamePieceState;
import frc.robot.subsystems.drive.Swerve;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.intake.Intake;

public class ScoreCommands {
    private final Swerve swerve;
    private final Intake intake;
    private final Superstructure superstructure;
    private final GamePieceState gamePieceState;

    public ScoreCommands(
            final Swerve swerve,
            final Intake intake,
            final Superstructure superstructure,
            final GamePieceState gamePieceState
    ) {
        this.swerve = swerve;
        this.intake = intake;
        this.superstructure = superstructure;
        this.gamePieceState = gamePieceState;
    }


}
