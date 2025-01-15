package frc.robot.state;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.intake.Intake;
import frc.robot.utils.subsystems.VirtualSubsystem;
import org.littletonrobotics.junction.Logger;

public class GamePieceState extends VirtualSubsystem {
    protected static final String LogKey = "GamePieceState";

    private final Intake intake;

    public enum State {
        NONE,
        INTAKING,
        HOLDING,
        SCORING
    }

    private State coralState = State.NONE;
    private State algaeState = State.NONE;

    public final Trigger isCoralNone = isStateTrigger(coralState, State.NONE);
    public final Trigger isCoralIntaking = isStateTrigger(coralState, State.INTAKING);
    public final Trigger isCoralHeld = isStateTrigger(coralState, State.HOLDING);
    public final Trigger isCoralScoring = isStateTrigger(coralState, State.SCORING);

    public final Trigger isAlgaeNone = isStateTrigger(algaeState, State.NONE);
    public final Trigger isAlgaeIntaking = isStateTrigger(algaeState, State.INTAKING);
    public final Trigger isAlgaeHeld = isStateTrigger(algaeState, State.HOLDING);
    public final Trigger isAlgaeScoring = isStateTrigger(algaeState, State.SCORING);

    public final Trigger hasCoral = isCoralHeld.or(isCoralScoring);
    public final Trigger hasAlgae = isAlgaeHeld.or(isAlgaeScoring);

    public GamePieceState(final Intake intake) {
        this.intake = intake;

        configureStateTriggers();
    }

    @Override
    public void periodic() {
        Logger.recordOutput(LogKey + "/CoralState", coralState.toString());
        Logger.recordOutput(LogKey + "/AlgaeState", algaeState.toString());

        Logger.recordOutput(LogKey + "/HasCoral", hasCoral.getAsBoolean());
        Logger.recordOutput(LogKey + "/HasAlgae", hasAlgae.getAsBoolean());
    }

    public Trigger isStateTrigger(final State currentState, final State state) {
        return new Trigger(() -> currentState == state);
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
        intake.isCoralIntaking.and(intake.isCoralPresent).onTrue(
                Commands.parallel(
                        setCoralState(State.HOLDING),
                        intake.runCoralRollerVoltage(-5)
                )
        );
        intake.isCoralOuttaking.and(isCoralHeld).onTrue(setCoralState(State.SCORING));
        intake.isCoralOuttaking.and(intake.isCoralPresent.negate()).onTrue(setCoralState(State.NONE));

        intake.isAlgaeIntaking.and(hasAlgae.negate()).onTrue(setAlgaeState(State.INTAKING));
        intake.isAlgaeIntaking.negate().and(isAlgaeIntaking).onTrue(setAlgaeState(State.NONE));
        intake.isAlgaeIntaking.and(intake.isAlgaePresent).onTrue(
                Commands.parallel(
                        setAlgaeState(State.HOLDING),
                        intake.runAlgaeRollerVoltage(-5)
                )
        );
        intake.isAlgaeOuttaking.and(isAlgaeHeld).onTrue(setAlgaeState(State.SCORING));
        intake.isAlgaeOuttaking.and(intake.isAlgaePresent.negate()).onTrue(setAlgaeState(State.NONE));
    }
}
