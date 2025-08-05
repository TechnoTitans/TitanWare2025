package frc.robot.state;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.Constants;
import frc.robot.subsystems.ground.intake.GroundIntake;
import frc.robot.utils.subsystems.VirtualSubsystem;
import org.littletonrobotics.junction.Logger;

import java.util.concurrent.ThreadLocalRandom;
import java.util.function.Supplier;

public class GroundIntakeGamepieceState extends VirtualSubsystem {
    protected static final String LogKey = "GroundIntakeGamepieceState";

    private final GroundIntake groundIntake;

    public enum State {
        NONE,
        INTAKING,
        HOLDING,
        TRANSFERRING
    }

    private State coralState = State.NONE;

    public final Trigger isCoralNone = isStateTrigger(() -> coralState, State.NONE);
    public final Trigger isCoralIntaking = isStateTrigger(() -> coralState, State.INTAKING);
    public final Trigger isCoralHolding = isStateTrigger(() -> coralState, State.HOLDING);
    public final Trigger isCoralTransferring = isStateTrigger(() -> coralState, State.TRANSFERRING);

    public final Trigger hasCoral = isCoralHolding.or(isCoralTransferring);

    public GroundIntakeGamepieceState(final Constants.RobotMode mode, final GroundIntake groundIntake) {
        this.groundIntake = groundIntake;

        configureStateTriggers();
        if (mode != Constants.RobotMode.REAL) {
            configureSimStateTriggers();
        }
    }

    @Override
    public void periodic() {
        Logger.recordOutput(LogKey + "/CoralState", coralState.toString());

        Logger.recordOutput(LogKey + "/IsCoralNone", isCoralNone.getAsBoolean());
        Logger.recordOutput(LogKey + "/IsCoralIntaking", isCoralIntaking.getAsBoolean());
        Logger.recordOutput(LogKey + "/IsCoralHeld", isCoralHolding.getAsBoolean());
        Logger.recordOutput(LogKey + "/IsCoralTransferring", isCoralTransferring.getAsBoolean());

        Logger.recordOutput(LogKey + "/HasCoral", hasCoral.getAsBoolean());
    }

    public Trigger isStateTrigger(final Supplier<State> currentState, final State state) {
        return new Trigger(() -> currentState.get() == state);
    }

    public Command setCoralState(final State coralState) {
        return Commands.runOnce(() -> this.coralState = coralState)
                .withName("GameStateSetCoralState: " + coralState.toString());
    }

    public State getCoralState() {
        return coralState;
    }

    public void configureStateTriggers() {
        groundIntake.isCoralIntaking.and(groundIntake.isCoralPresent.negate())
                .onTrue(Commands.parallel(
                        setCoralState(State.INTAKING)
                ));
        groundIntake.isCoralIntaking.negate().and(isCoralIntaking).onTrue(setCoralState(State.NONE));
        groundIntake.isCoralPresent.onTrue(setCoralState(State.HOLDING));

        isCoralHolding.onTrue(groundIntake.holdCoral());

        groundIntake.isCoralOuttaking.and(isCoralHolding).onTrue(setCoralState(State.TRANSFERRING));
        groundIntake.isCoralPresent.negate()
                .onTrue(setCoralState(State.NONE));
        groundIntake.isCoralOuttaking.negate().and(isCoralTransferring).and(groundIntake.isCoralPresent)
                .onTrue(setCoralState(State.HOLDING));
    }

    private Command waitRand(
            final ThreadLocalRandom random,
            final double lowerInclusiveSeconds,
            final double upperExclusiveSeconds
    ) {
        return Commands.waitSeconds(random.nextDouble(lowerInclusiveSeconds, upperExclusiveSeconds));
    }

    private Command setCANRangeDistanceCommand(final double gamepieceDistanceMeters) {
        return Commands.runOnce(() -> groundIntake.setTOFDistance(gamepieceDistanceMeters));
    }

    public void configureSimStateTriggers() {
        final ThreadLocalRandom random = ThreadLocalRandom.current();

        groundIntake.isCoralIntaking.and(hasCoral.negate()).whileTrue(
                Commands.sequence(
                        waitRand(random, 0.5, 0.75),
                        setCANRangeDistanceCommand(0.1)
                )
        );

        groundIntake.isCoralOuttaking.and(hasCoral).whileTrue(
                Commands.sequence(
                        waitRand(random, 0.1, 0.25),
                        setCANRangeDistanceCommand(0.5)
                )
        );
    }
}
