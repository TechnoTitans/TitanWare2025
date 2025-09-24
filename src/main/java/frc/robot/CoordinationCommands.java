package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.endeffector.Intake;
import frc.robot.subsystems.intake.ground.GroundIntake;
import frc.robot.subsystems.superstructure.Superstructure;

public class CoordinationCommands {
    private CoordinationCommands() {}

    public static Command handOff(
            final Superstructure superstructure,
            final Intake intake,
            final GroundIntake groundIntake
    ) {
        return Commands.deadline(
                Commands.sequence(
                        Commands.waitUntil(superstructure.atSetpoint(Superstructure.Goal.HANDOFF)),
                        groundIntake.handOff()
                                .asProxy()
                                .until(intake.isCoralPresent)
                                .withTimeout(5)
                ),
                superstructure.toGoal(Superstructure.Goal.HANDOFF),
                groundIntake.hold().asProxy(),
                intake.handOff()
        )
                .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming)
                .withName("CoordinationHandOff");
    }
}
