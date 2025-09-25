package frc.robot.state;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.CoordinationCommands;
import frc.robot.constants.Constants;
import frc.robot.subsystems.intake.endeffector.Intake;
import frc.robot.subsystems.intake.ground.GroundIntake;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.utils.commands.LoggedTrigger;
import frc.robot.utils.subsystems.VirtualSubsystem;
import org.littletonrobotics.junction.Logger;

import java.util.concurrent.ThreadLocalRandom;
import java.util.function.Supplier;

public class GamepieceState extends VirtualSubsystem {
    protected static final String LogKey = "GamepieceState";

    private final Superstructure superstructure;
    private final Intake intake;
    private final GroundIntake groundIntake;

    public enum IntakeState {
        NONE,
        INTAKING,
        HOLDING,
        SCORING
    }

    public enum GroundIntakeState {
        NONE,
        INTAKING,
        HANDING_OFF
    }

    private IntakeState coralState = IntakeState.NONE;
    private IntakeState algaeState = IntakeState.NONE;
    private GroundIntakeState groundState = GroundIntakeState.NONE;

    private final LoggedTrigger.Group group = LoggedTrigger.Group.from(LogKey);

    public final LoggedTrigger isCoralNone = isStateTrigger(() -> coralState, IntakeState.NONE);
    public final LoggedTrigger isCoralIntaking = isStateTrigger(() -> coralState, IntakeState.INTAKING);
    public final LoggedTrigger isCoralHolding = isStateTrigger(() -> coralState, IntakeState.HOLDING);
    public final LoggedTrigger isCoralScoring = isStateTrigger(() -> coralState, IntakeState.SCORING);

    public final LoggedTrigger isAlgaeNone = isStateTrigger(() -> algaeState, IntakeState.NONE);
    public final LoggedTrigger isAlgaeIntaking = isStateTrigger(() -> algaeState, IntakeState.INTAKING);
    public final LoggedTrigger isAlgaeHolding = isStateTrigger(() -> algaeState, IntakeState.HOLDING);
    public final LoggedTrigger isAlgaeScoring = isStateTrigger(() -> algaeState, IntakeState.SCORING);

    public final LoggedTrigger isGroundNone = isStateTrigger(() -> groundState, GroundIntakeState.NONE);
    public final LoggedTrigger isGroundIntaking = isStateTrigger(() -> groundState, GroundIntakeState.INTAKING);
    public final LoggedTrigger isGroundHandingOff = isStateTrigger(() -> groundState, GroundIntakeState.HANDING_OFF);

    public final LoggedTrigger intakeHasCoral = isCoralHolding.or(isCoralScoring);
    public final LoggedTrigger intakeHasAlgae = isAlgaeHolding.or(isAlgaeScoring);
    public final LoggedTrigger groundHasCoral = isGroundHandingOff;

    public GamepieceState(
            final Constants.RobotMode mode,
            final Superstructure superstructure,
            final Intake intake,
            final GroundIntake groundIntake
    ) {
        this.superstructure = superstructure;
        this.intake = intake;
        this.groundIntake = groundIntake;

        configureStateTriggers();
        if (mode != Constants.RobotMode.REAL) {
            configureSimStateTriggers();
        }
    }

    @Override
    public void periodic() {
        Logger.recordOutput(LogKey + "/CoralState", coralState.toString());
        Logger.recordOutput(LogKey + "/AlgaeState", algaeState.toString());
        Logger.recordOutput(LogKey + "/GroundState", groundState.toString());

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
        Logger.recordOutput(LogKey + "/IsGroundHandingOff", isGroundHandingOff.getAsBoolean());

        Logger.recordOutput(LogKey + "/IntakeHasCoral", intakeHasCoral.getAsBoolean());
        Logger.recordOutput(LogKey + "/IntakeHasAlgae", intakeHasAlgae.getAsBoolean());
        Logger.recordOutput(LogKey + "/IntakeHasAlgae", groundHasCoral.getAsBoolean());
    }

    public LoggedTrigger isStateTrigger(final Supplier<IntakeState> currentState, final IntakeState state) {
        return group.t(String.format("isState(%s)", state), () -> currentState.get() == state);
    }

    public LoggedTrigger isStateTrigger(final Supplier<GroundIntakeState> currentState, final GroundIntakeState state) {
        return group.t(String.format("isState(%s)", state), () -> currentState.get() == state);
    }

    public Command setCoralState(final IntakeState coralState) {
        return Commands.runOnce(() -> this.coralState = coralState)
                .withName("GamePieceStateSetCoralState: " + coralState.toString());
    }

    public Command setAlgaeState(final IntakeState algaeState) {
        return Commands.runOnce(() -> this.algaeState = algaeState)
                .withName("GamePieceStateSetAlgaeState: " + algaeState.toString());
    }

    public Command setGroundState(final GroundIntakeState groundState) {
        return Commands.runOnce(() -> this.groundState = groundState)
                .withName("GamePieceStateSetGroundState: " + groundState.toString());
    }

    public void configureStateTriggers() {
        intake.isAlgaeIntaking.negate().and(intake.isCoralIntaking).and(intake.isCoralPresent.negate())
                .onTrue(Commands.parallel(
                        setCoralState(IntakeState.INTAKING),
                        setAlgaeState(IntakeState.NONE)
                ).withName("GamePieceStateSetCoralIntaking"));
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
                ).withName("GamePieceStateSetAlgaeIntaking"));
        isCoralNone.and(intake.isAlgaeIntaking.negate()).and(isAlgaeIntaking).onTrue(setAlgaeState(IntakeState.NONE));
        isCoralNone.and(intake.isCurrentAboveAlgaeThreshold).and(intake.isAlgaeIntaking)
                .onTrue(setAlgaeState(IntakeState.HOLDING));

        intake.isCoralOuttaking.and(isAlgaeHolding).onTrue(setAlgaeState(IntakeState.NONE));

        isAlgaeHolding.onTrue(intake.holdAlgae());

        isCoralNone.and(intake.isAlgaeOuttaking).and(isAlgaeHolding).onTrue(setAlgaeState(IntakeState.SCORING));
        isCoralNone.and(intake.isAlgaeOuttaking).and(intake.isCurrentAboveAlgaeThreshold.negate()).onTrue(setAlgaeState(IntakeState.NONE));
        isCoralNone.and(intake.isAlgaeOuttaking.negate()).and(isAlgaeScoring).and(intake.isCurrentAboveAlgaeThreshold)
                .onTrue(setAlgaeState(IntakeState.HOLDING));

        groundIntake.isIntaking.and(groundIntake.isCoralPresent.negate())
                .onTrue(setGroundState(GroundIntakeState.INTAKING));
        groundIntake.isIntaking.negate().and(isGroundIntaking).onTrue(setGroundState(GroundIntakeState.NONE));

        groundIntake.isCoralPresent.onTrue(setGroundState(GroundIntakeState.HANDING_OFF));

        isGroundHandingOff.onTrue(CoordinationCommands.handOff(superstructure, intake, groundIntake));
//
        groundIntake.isCoralPresent.negate()
                .onTrue(setGroundState(GroundIntakeState.NONE));
    }

    @SuppressWarnings("SameParameterValue")
    private Command waitRand(
            final ThreadLocalRandom random,
            final double lowerInclusiveSeconds,
            final double upperExclusiveSeconds
    ) {
        return Commands.waitSeconds(random.nextDouble(lowerInclusiveSeconds, upperExclusiveSeconds));
    }

    private Command setIntakeCANRangeDistanceCommand(final double distanceMeters) {
        return Commands.runOnce(() -> intake.setTOFDistance(distanceMeters));
    }

    private Command setGroundCANRangeDistanceCommand(final double distanceMeters) {
        return Commands.runOnce(() -> groundIntake.setCoralCANRangeDistance(distanceMeters));
    }

    public void configureSimStateTriggers() {
        final ThreadLocalRandom random = ThreadLocalRandom.current();

        intake.isCoralIntaking.and(intakeHasCoral.negate()).whileTrue(
                Commands.sequence(
                        waitRand(random, 0.5, 0.75),
                        setIntakeCANRangeDistanceCommand(0.1)
                ).withName("GamePieceStateSimIntakeCoral")
        );
        intake.isCoralOuttaking.and(intakeHasCoral).whileTrue(
                Commands.sequence(
                        waitRand(random, 0.1, 0.25),
                        setIntakeCANRangeDistanceCommand(0.5)
                ).withName("GamePieceStateSimOuttakeCoral")
        );

        groundIntake.isIntaking.and(groundHasCoral.negate()).whileTrue(
                Commands.sequence(
                        waitRand(random, 0.75, 1.25),
                        setGroundCANRangeDistanceCommand(0.06)
                ).withName("GamePieceStateSimGroundIntakeCoral")
        );
        groundIntake.isOuttaking.and(groundHasCoral).and(intake.isCoralHandingOff.negate()).whileTrue(
                Commands.sequence(
                        waitRand(random, 0.2, 0.3),
                        setGroundCANRangeDistanceCommand(0.5)
                ).withName("GamePieceStateSimGroundOuttakeCoral")
        );

        groundIntake.isOuttaking.and(groundHasCoral).and(intake.isCoralHandingOff).whileTrue(
                Commands.sequence(
                        waitRand(random, 0.05, 0.1),
                        Commands.runOnce(() -> {
                            intake.setTOFDistance(0.19);
                            groundIntake.setCoralCANRangeDistance(0.5);
                        })
                ).withName("GamePieceStateSimHandoffCoral")
        );
    }
}
