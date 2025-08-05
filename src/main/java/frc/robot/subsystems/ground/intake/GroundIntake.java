package frc.robot.subsystems.ground.intake;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.Constants;
import frc.robot.constants.HardwareConstants;
import org.littletonrobotics.junction.Logger;

import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class GroundIntake extends SubsystemBase {

    public enum Transfer_Mode {
        TRANSFER_UNTIL_NO_CORAL,
        TRANSFER_FOR_TIME
    }
    protected static final String LogKey = "GroundIntake";
    private static final double NoCoralTOFReading = 0.4;

    private final GroundIntakeIO groundIntakeIO;
    private final GroundIntakeIOInputsAutoLogged inputs;

    private boolean coralIntaking = false;
    private boolean coralOuttaking = false;

    private double rollerVelocitySetpoint = 0.0;
    private double rollerVoltageSetpoint = 0.0;
    private double rollerTorqueCurrentSetpoint = 0.0;

    public final Trigger isCoralIntaking;
    public final Trigger isCoralOuttaking;
    public final Trigger isCoralIntakeStopped;

    public final Trigger isCoralPresent;

    public GroundIntake(final Constants.RobotMode mode, final HardwareConstants.GroundIntakeConstants constants) {
        this.groundIntakeIO = switch (mode) {
            case REAL ->  new GroundIntakeIOReal(constants);
            case SIM -> new GroundIntakeIOSim(constants);
            case REPLAY, DISABLED -> new GroundIntakeIO() {};
        };

        this.inputs = new GroundIntakeIOInputsAutoLogged();

        this.isCoralIntaking = new Trigger(() -> coralIntaking);
        this.isCoralOuttaking = new Trigger(() -> coralOuttaking);
        this.isCoralIntakeStopped = isCoralIntaking.negate().and(isCoralOuttaking.negate());

        this.isCoralPresent = new Trigger(this::isCoralPresent);

        this.groundIntakeIO.config();
    }

    @Override
    public void periodic() {
        groundIntakeIO.updateInputs(inputs);
        Logger.processInputs(LogKey, inputs);

        Logger.recordOutput(LogKey + "/RollerVelocitySetpoint", rollerVelocitySetpoint);
        Logger.recordOutput(LogKey + "/RollerVoltageSetpoint", rollerVoltageSetpoint);
        Logger.recordOutput(LogKey + "/RollerTorqueCurrentSetpoint", rollerTorqueCurrentSetpoint);

        Logger.recordOutput(LogKey + "/Trigger/IsCoralPresent", isCoralPresent);
        Logger.recordOutput(LogKey + "/Trigger/IsCoralIntaking", isCoralIntaking);
        Logger.recordOutput(LogKey + "/Trigger/IsCoralOuttaking", isCoralOuttaking);
        Logger.recordOutput(LogKey + "/Trigger/IsCoralIntakeStopped", isCoralIntakeStopped);

        Logger.recordOutput(LogKey + "/Trigger/CoralDistanceMeters", getCoralDistanceMeters());
    }

    private double getCoralDistanceMeters() {
        return inputs.coralTOFDistanceMeters;
    }
    public boolean isCoralPresent() {
        return getCoralDistanceMeters() < NoCoralTOFReading;
    }

    public Command intakeCoralGround() {
        return Commands.sequence(
                runOnce(() -> this.coralIntaking = true),
                toRollerVelocity(10.0) //TODO: Change Number,
        )
                .finallyDo(() -> this.coralIntaking = false)
                .withName("IntakeCoralGround");
    }

    public Command holdCoral() {
        return toInstantRollerTorqueCurrent(12)
                .withName("HoldGroundCoral");
    }

    public Command transferCoral(final Supplier<Transfer_Mode> transferModeSupplier, final Trigger isCoralTransferred) {
        return Commands.sequence(
                runOnce(() -> this.coralOuttaking = true),
                Commands.select(
                        Map.of(
                                Transfer_Mode.TRANSFER_UNTIL_NO_CORAL,
                                    Commands.sequence(
                                            toInstantRollerVoltage(-9),
                                            Commands.waitUntil(isCoralTransferred)
                                                    .withTimeout(2)
                                    ),
                                Transfer_Mode.TRANSFER_FOR_TIME,
                                    Commands.sequence(
                                            toInstantRollerVoltage(-9),
                                            Commands.waitSeconds(0.2)
                                    )
                        ),
                        transferModeSupplier
                ),
                instantStopCommand()
        )
                .finallyDo(() -> this.coralOuttaking = false)
                .withName("GroundTransferCoral");
    }

    public Command scoreL1() {
        return Commands.sequence(
                runOnce(() -> this.coralOuttaking = true),
                toInstantRollerVoltage(-9),
                Commands.waitUntil(isCoralPresent.negate())
                        .withTimeout(2)
        )
                .finallyDo(() -> this.coralOuttaking = false)
                .withName("GroundScoreL1");
    }

    public Command ejectCoral() {
        return Commands.sequence(
                runOnce(() -> this.coralOuttaking = true),
                toInstantRollerVoltage(-10) //TODO: Change Number

        )
                .finallyDo(() -> this.coralOuttaking = false)
                .withName("EjectCoralGround");
    }

    public Command toRollerVelocity(final double velocityRotsPerSec) {
        return runEnd(
                () -> {
                    this.rollerVelocitySetpoint = velocityRotsPerSec;
                    groundIntakeIO.toRollerVelocity(rollerVelocitySetpoint);
                },
                () -> {
                    this.rollerVelocitySetpoint = 0.0;
                    groundIntakeIO.toRollerVelocity(rollerVelocitySetpoint);
                }
        ).withName("ToRollerVelocity");
    }

    public Command toInstantRollerVoltage(final double volts) {
        return runOnce(
                () -> {
                    this.rollerVoltageSetpoint = volts;
                    groundIntakeIO.toRollerVoltage(rollerVoltageSetpoint);
                }
        ).withName("ToInstantRollerVoltage");
    }

    public Command toInstantRollerTorqueCurrent(final double torqueCurrentAmps) {
        return runOnce(
                () -> {
                    this.rollerTorqueCurrentSetpoint = torqueCurrentAmps;
                    groundIntakeIO.toRollerTorqueCurrent(torqueCurrentAmps);
                }
        ).withName("ToInstantRollerTorqueCurrent");
    }

    public Command instantStopCommand() {
        return Commands.runOnce(() -> {
                    this.coralIntaking = false;
                    this.coralOuttaking = false;
                    this.rollerVelocitySetpoint = 0.0;
                    this.rollerVoltageSetpoint = 0.0;
                    groundIntakeIO.toRollerVoltage(0);
                }
        ).withName("InstantStop");
    }

    public void setTOFDistance(final double distanceMeters) {
        groundIntakeIO.setTOFDistance(distanceMeters);
    }

}
