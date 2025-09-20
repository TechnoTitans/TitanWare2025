package frc.robot.utils.commands;

import edu.wpi.first.wpilibj2.command.Command;

public class FastCommands {
    public static Command sequence(final Command... commands) {
        return new FastSequentialCommandGroup(commands);
    }
}
