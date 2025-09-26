package frc.robot.state;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.Constants;
import frc.robot.subsystems.ground.GroundIntake;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.superstructure.ground.GroundIntakeArm;
import frc.robot.utils.subsystems.VirtualSubsystem;
import org.littletonrobotics.junction.Logger;

import java.util.concurrent.ThreadLocalRandom;
import java.util.function.Supplier;

public class IntakeGamepieceState extends VirtualSubsystem {
    protected static final String LogKey = "IntakeGamepieceState";

    private final Intake intake;
    private final GroundIntake groundIntake;
    private final GroundIntakeArm groundIntakeArm;

    public enum IntakeState {
        NONE,
        INTAKING,
        HOLDING,
        SCORING
    }

    public enum GroundState {
        NONE,
        INTAKING,
        HOLDING,
        SCORING,
        HANDING_OFF
    }

    private IntakeState coralState = IntakeState.NONE;
    private IntakeState algaeState = IntakeState.NONE;
    private GroundState groundState = GroundState.NONE;

    public final Trigger isCoralNone = isStateTrigger(() -> coralState, IntakeState.NONE);
    public final Trigger isCoralIntaking = isStateTrigger(() -> coralState, IntakeState.INTAKING);
    public final Trigger isCoralHolding = isStateTrigger(() -> coralState, IntakeState.HOLDING);
    public final Trigger isCoralScoring = isStateTrigger(() -> coralState, IntakeState.SCORING);

    public final Trigger isAlgaeNone = isStateTrigger(() -> algaeState, IntakeState.NONE);
    public final Trigger isAlgaeIntaking = isStateTrigger(() -> algaeState, IntakeState.INTAKING);
    public final Trigger isAlgaeHolding = isStateTrigger(() -> algaeState, IntakeState.HOLDING);
    public final Trigger isAlgaeScoring = isStateTrigger(() -> algaeState, IntakeState.SCORING);

    public final Trigger isGroundNone = isStateTrigger(() -> groundState, GroundState.NONE);
    public final Trigger isGroundIntaking = isStateTrigger(() -> groundState, GroundState.INTAKING);
    public final Trigger isGroundHolding = isStateTrigger(() -> groundState, GroundState.HOLDING);
    public final Trigger isGroundScoring = isStateTrigger(() -> groundState, GroundState.SCORING);
    public final Trigger isGroundHandingOff = isStateTrigger(() -> groundState, GroundState.HANDING_OFF);

    public final Trigger hasCoral = isCoralHolding.or(isCoralScoring);
    public final Trigger hasGroundCoral = isGroundHolding.or(isGroundHandingOff).or(isGroundScoring);
    public final Trigger hasAlgae = isAlgaeHolding.or(isAlgaeScoring);

    public IntakeGamepieceState(
        final Constants.RobotMode mode,
        final Intake intake,
        final GroundIntake groundIntake,
        final GroundIntakeArm groundIntakeArm
    ) {
        this.intake = intake;
        this.groundIntake = groundIntake;
        this.groundIntakeArm = groundIntakeArm;

        configureStateTriggers();
        if (mode != Constants.RobotMode.REAL) {
            configureSimStateTriggers();
        }
    }

    @Override
    public void periodic() {
        Logger.recordOutput(LogKey + "/CoralState", coralState.toString());
        Logger.recordOutput(LogKey + "/GroundCoralState", groundState.toString());
        Logger.recordOutput(LogKey + "/AlgaeState", algaeState.toString());

        Logger.recordOutput(LogKey + "/IsCoralNone", isCoralNone.getAsBoolean());
        Logger.recordOutput(LogKey + "/IsCoralIntaking", isCoralIntaking.getAsBoolean());
        Logger.recordOutput(LogKey + "/IsCoralHeld", isCoralHolding.getAsBoolean());
        Logger.recordOutput(LogKey + "/IsCoralScoring", isCoralScoring.getAsBoolean());

        Logger.recordOutput(LogKey + "/IsAlgaeNone", isAlgaeNone.getAsBoolean());
        Logger.recordOutput(LogKey + "/IsAlgaeIntaking", isAlgaeIntaking.getAsBoolean());
        Logger.recordOutput(LogKey + "/IsAlgaeHeld", isAlgaeHolding.getAsBoolean());
        Logger.recordOutput(LogKey + "/IsAlgaeScoring", isAlgaeScoring.getAsBoolean());

        Logger.recordOutput(LogKey + "/IsGroundNone", isGroundNone.getAsBoolean());
        Logger.recordOutput(LogKey + "/IsGroundIntaking", isGroundIntaking.getAsBoolean());
        Logger.recordOutput(LogKey + "/IsGroundHeld", isGroundHolding.getAsBoolean());
        Logger.recordOutput(LogKey + "/IsGroundScoring", isGroundScoring.getAsBoolean());
        Logger.recordOutput(LogKey + "/IsGroundHandingOff", isGroundHandingOff.getAsBoolean());

        Logger.recordOutput(LogKey + "/HasCoral", hasCoral.getAsBoolean());
        Logger.recordOutput(LogKey + "/HasGroundCoral", hasGroundCoral.getAsBoolean());
        Logger.recordOutput(LogKey + "/HasAlgae", hasAlgae.getAsBoolean());
    }

    public Trigger isStateTrigger(final Supplier<IntakeState> currentState, final IntakeState state) {
        return new Trigger(() -> currentState.get() == state);
    }

    public Trigger isStateTrigger(final Supplier<GroundState> currentState, final GroundState state) {
        return new Trigger(() -> currentState.get() == state);
    }


    public Command setCoralState(final IntakeState coralState) {
        return Commands.runOnce(() -> this.coralState = coralState)
                .withName("GameStateSetCoralState: " + coralState.toString());
    }

    public Command setAlgaeState(final IntakeState algaeState) {
        return Commands.runOnce(() -> this.algaeState = algaeState)
                .withName("GameStateSetAlgaeState: " + algaeState.toString());
    }

    public Command setGroundState(final GroundState groundState) {
        return Commands.runOnce(() -> this.groundState = groundState)
                .withName("GameStateSetGroundState: " + groundState.toString());
    }

    @SuppressWarnings("unused")
    public IntakeState getCoralIntake() {
        return coralState;
    }

    @SuppressWarnings("unused")
    public IntakeState getAlgaeState() {
        return algaeState;
    }

    public void configureStateTriggers() {
        intake.isAlgaeIntaking.negate().and(intake.isCoralIntaking).and(intake.isCoralPresent.negate())
                .onTrue(Commands.parallel(
                        setCoralState(IntakeState.INTAKING),
                        setAlgaeState(IntakeState.NONE)
                ));
        intake.isCoralIntaking.negate().and(isCoralIntaking).onTrue(setCoralState(IntakeState.NONE));
        intake.isCoralPresent.onTrue(setCoralState(IntakeState.HOLDING));

        isCoralHolding.onTrue(intake.holdCoral());

        intake.isCoralOuttaking.and(isCoralHolding).onTrue(setCoralState(IntakeState.SCORING));
        intake.isCoralPresent.negate()
                .onTrue(setCoralState(IntakeState.NONE));
        intake.isCoralOuttaking.negate().and(isCoralScoring).and(intake.isCoralPresent)
                .onTrue(setCoralState(IntakeState.HOLDING));

        intake.isAlgaeIntaking.and(intake.isCurrentAboveAlgaeThreshold.negate()).onTrue(
                Commands.parallel(
                        setAlgaeState(IntakeState.INTAKING),
                        setCoralState(IntakeState.NONE)
                ));
        isCoralNone.and(intake.isAlgaeIntaking.negate()).and(isAlgaeIntaking).onTrue(setAlgaeState(IntakeState.NONE));
        isCoralNone.and(intake.isCurrentAboveAlgaeThreshold).and(intake.isAlgaeIntaking)
                .onTrue(setAlgaeState(IntakeState.HOLDING));

        intake.isCoralOuttaking.and(isAlgaeHolding).onTrue(setAlgaeState(IntakeState.NONE));

        isAlgaeHolding.onTrue(intake.holdAlgae());

        isCoralNone.and(intake.isAlgaeOuttaking).and(isAlgaeHolding).onTrue(setAlgaeState(IntakeState.SCORING));
        isCoralNone.and(intake.isAlgaeOuttaking).and(intake.isCurrentAboveAlgaeThreshold.negate()).onTrue(setAlgaeState(IntakeState.NONE));
        isCoralNone.and(intake.isAlgaeOuttaking.negate()).and(isAlgaeScoring).and(intake.isCurrentAboveAlgaeThreshold)
                .onTrue(setAlgaeState(IntakeState.HOLDING));

        groundIntake.isCoralIntaking.and(groundIntake.isCoralPresent.negate()).onTrue(setGroundState(GroundState.INTAKING));
        groundIntake.isCoralIntaking.negate().and(isGroundIntaking).onTrue(setGroundState(GroundState.NONE));

        groundIntake.isCoralPresent.onTrue(setGroundState(GroundState.HOLDING));

        isGroundHolding.onTrue(groundIntake.holdCoral());

        groundIntake.isCoralOuttaking.and(isGroundHolding).onTrue(setGroundState(GroundState.HANDING_OFF));
        groundIntake.isCoralPresent.negate().onTrue(setGroundState(GroundState.NONE));
        // groundIntake.isCoralOuttaking.negate().and(isCoralTransferring).and(groundIntake.isCoralPresent)
                // .onTrue(setGroundState(State.HOLDING));
    }

    @SuppressWarnings("SameParameterValue")
    private Command waitRand(
            final ThreadLocalRandom random,
            final double lowerInclusiveSeconds,
            final double upperExclusiveSeconds
    ) {
        return Commands.waitSeconds(random.nextDouble(lowerInclusiveSeconds, upperExclusiveSeconds));
    }

    private Command setCANRangeDistanceCommand(final double gamepieceDistanceMeters) {
        return Commands.runOnce(() -> intake.setTOFDistance(gamepieceDistanceMeters));
    }

    private Command setGroundCANRangeDistanceCommand(final double gamepieceDistanceMeters) {
        return Commands.runOnce(() -> groundIntake.setTOFDistance(gamepieceDistanceMeters));
    }

    public void configureSimStateTriggers() {
        final ThreadLocalRandom random = ThreadLocalRandom.current();

        intake.isCoralIntaking.and(hasCoral.negate()).whileTrue(
                Commands.sequence(
                        waitRand(random, 0.5, 0.75),
                        setCANRangeDistanceCommand(0.1)
                )
        );

        intake.isCoralOuttaking.and(hasCoral).whileTrue(
                Commands.sequence(
                        waitRand(random, 0.1, 0.25),
                        setCANRangeDistanceCommand(0.5)
                )
        );

        groundIntake.isCoralIntaking.and(hasGroundCoral.negate()).whileTrue(
                Commands.sequence(
                        waitRand(random, 0.5, 0.75),
                        setGroundCANRangeDistanceCommand(0.1)
                )
        );

        groundIntake.isCoralOuttaking.and(hasGroundCoral).and(groundIntakeArm.atSetpoint).whileTrue(
                Commands.sequence(
                        waitRand(random, 0.1, 0.25),
                        setGroundCANRangeDistanceCommand(0.5)
                )
        );
    }
}
