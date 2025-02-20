package frc.robot.state;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.Constants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.utils.subsystems.VirtualSubsystem;
import org.littletonrobotics.junction.Logger;

import java.util.concurrent.ThreadLocalRandom;
import java.util.function.Supplier;

public class GamepieceState extends VirtualSubsystem {
    protected static final String LogKey = "GamepieceState";

    private final Intake intake;

    public enum State {
        NONE,
        INTAKING,
        HOLDING,
        SCORING
    }

    private State coralState = State.NONE;
    private State algaeState = State.NONE;

    public final Trigger isCoralNone = isStateTrigger(() -> coralState, State.NONE);
    public final Trigger isCoralIntaking = isStateTrigger(() -> coralState, State.INTAKING);
    public final Trigger isCoralHeld = isStateTrigger(() -> coralState, State.HOLDING);
    public final Trigger isCoralScoring = isStateTrigger(() -> coralState, State.SCORING);

    public final Trigger isAlgaeNone = isStateTrigger(() -> algaeState, State.NONE);
    public final Trigger isAlgaeIntaking = isStateTrigger(() -> algaeState, State.INTAKING);
    public final Trigger isAlgaeHeld = isStateTrigger(() -> algaeState, State.HOLDING);
    public final Trigger isAlgaeScoring = isStateTrigger(() -> algaeState, State.SCORING);

    public final Trigger hasCoral = isCoralHeld.or(isCoralScoring);
    public final Trigger hasAlgae = isAlgaeHeld.or(isAlgaeScoring);

    public GamepieceState(final Constants.RobotMode mode, final Intake intake) {
        this.intake = intake;

        configureStateTriggers();
        if (mode != Constants.RobotMode.REAL) {
            configureSimStateTriggers();
        }
    }

    @Override
    public void periodic() {
        Logger.recordOutput(LogKey + "/CoralState", coralState.toString());
        Logger.recordOutput(LogKey + "/AlgaeState", algaeState.toString());

        Logger.recordOutput(LogKey + "/IsCoralNone", isCoralNone.getAsBoolean());
        Logger.recordOutput(LogKey + "/IsCoralIntaking", isCoralIntaking.getAsBoolean());
        Logger.recordOutput(LogKey + "/IsCoralHeld", isCoralHeld.getAsBoolean());
        Logger.recordOutput(LogKey + "/IsCoralScoring", isCoralScoring.getAsBoolean());

        Logger.recordOutput(LogKey + "/IsAlgaeNone", isAlgaeNone.getAsBoolean());
        Logger.recordOutput(LogKey + "/IsAlgaeIntaking", isAlgaeIntaking.getAsBoolean());
        Logger.recordOutput(LogKey + "/IsAlgaeHeld", isAlgaeHeld.getAsBoolean());
        Logger.recordOutput(LogKey + "/IsAlgaeScoring", isAlgaeScoring.getAsBoolean());

        Logger.recordOutput(LogKey + "/HasCoral", hasCoral.getAsBoolean());
        Logger.recordOutput(LogKey + "/HasAlgae", hasAlgae.getAsBoolean());
    }

    public Trigger isStateTrigger(final Supplier<State> currentState, final State state) {
        return new Trigger(() -> currentState.get() == state);
    }

    public Command setCoralState(final State coralState) {
        return Commands.runOnce(() -> this.coralState = coralState);
    }

    public Command setAlgaeState(final State algaeState) {
        return Commands.runOnce(() -> this.algaeState = algaeState);
    }

    public State getCoralState() {
        return coralState;
    }

    public State getAlgaeState() {
        return algaeState;
    }

    public void configureStateTriggers() {
        intake.isCoralIntaking.and(hasCoral.negate()).onTrue(setCoralState(State.INTAKING));
        intake.isCoralIntaking.negate().and(isCoralIntaking).onTrue(setCoralState(State.NONE));
        intake.isCoralPresent.whileTrue(
                Commands.parallel(
                        setCoralState(State.HOLDING),
                        intake.toCoralRollerVoltage(3)
                ).withName("GameStateCoralHold")
        );
        intake.isCoralOuttaking.and(isCoralHeld).onTrue(setCoralState(State.SCORING));
        intake.isCoralOuttaking.and(intake.isCoralPresent.negate()).onTrue(setCoralState(State.NONE));

        intake.isAlgaeIntaking.and(hasAlgae.negate()).onTrue(setAlgaeState(State.INTAKING));
        intake.isAlgaeIntaking.negate().and(isAlgaeIntaking).onTrue(setAlgaeState(State.NONE));
        intake.isAlgaePresent.whileTrue(
                Commands.parallel(
                        setAlgaeState(State.HOLDING),
                        intake.toAlgaeRollerVoltage(4)
                ).withName("GameStateAlgaeHold")
        );
        intake.isAlgaeOuttaking.and(isAlgaeHeld).onTrue(setAlgaeState(State.SCORING));
        intake.isAlgaeOuttaking.and(intake.isAlgaePresent.negate()).onTrue(setAlgaeState(State.NONE));
    }

    private Command waitRand(
            final ThreadLocalRandom random,
            final double lowerInclusiveSeconds,
            final double upperExclusiveSeconds
    ) {
        return Commands.waitSeconds(random.nextDouble(lowerInclusiveSeconds, upperExclusiveSeconds));
    }

    private Command setCANRangeDistanceCommand(final double gamepieceDistanceMeters) {
        return Commands.runOnce(() -> intake.setCANRangeDistance(gamepieceDistanceMeters));
    }

    public void configureSimStateTriggers() {
        final ThreadLocalRandom random = ThreadLocalRandom.current();

        intake.isCoralIntaking.and(hasCoral.negate()).whileTrue(
                Commands.sequence(
                    waitRand(random, 0.1, 2),
                    Commands.waitSeconds(0.15),
                    setCANRangeDistanceCommand(Units.inchesToMeters(1))
        ));

        intake.isCoralOuttaking.and(hasCoral).whileTrue(
                Commands.sequence(
                        waitRand(random, 0.11, 2.1),
                        Commands.waitSeconds(0.15),
                        setCANRangeDistanceCommand(10)
        ));

        intake.isAlgaeIntaking.and(hasAlgae.negate()).whileTrue(
                Commands.sequence(
                    waitRand(random, 0.1, 2),
                    Commands.waitSeconds(0.15),
                    setCANRangeDistanceCommand(Units.inchesToMeters(1))
        ));

        intake.isCoralOuttaking.and(hasCoral).whileTrue(
                Commands.sequence(
                        waitRand(random, 0.1, 2),
                        Commands.waitSeconds(0.15),
                        setCANRangeDistanceCommand(10)
        ));
    }
}
