package frc.robot.subsystems.ground;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.units.CurrentUnit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.Constants;
import frc.robot.constants.HardwareConstants;
import org.littletonrobotics.junction.Logger;

import java.util.function.Consumer;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.units.Units.Amp;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Seconds;

public class GroundIntake extends SubsystemBase {
    protected static final String LogKey = "GroundIntake";
    private static final double NoCoralTOFReading = 0.4;

    private final GroundIntakeIO groundIntakeIO;
    private final GroundIntakeIOInputsAutoLogged inputs;

    private final SysIdRoutine rollerVoltageSysIdRoutine;
    private final SysIdRoutine rollerTorqueCurrentSysIdRoutine;

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

        this.rollerVoltageSysIdRoutine = makeVoltageSysIdRoutine(
                Volts.of(2).per(Second),
                Volts.of(10),
                Seconds.of(10),
                groundIntakeIO::toRollerVoltage
        );
        this.rollerTorqueCurrentSysIdRoutine = makeTorqueCurrentSysIdRoutine(
                Amps.of(4).per(Second),
                Amp.of(40),
                Seconds.of(10),
                groundIntakeIO::toRollerTorqueCurrent
        );

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

    public Command handoffCoral() {
        return Commands.sequence(
                runOnce(() -> this.coralOuttaking = true),
                runRollerVoltage(-9),
                instantStopCommand()
        )
                .finallyDo(() -> this.coralOuttaking = false)
                .withName("GroundHandoffCoral");
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

    public Command runRollerVoltage(final double volts) {
        return run(
                () -> {
                    this.rollerVoltageSetpoint = volts;
                    groundIntakeIO.toRollerVoltage(rollerVoltageSetpoint);
                }
        ).withName("RunRollerVoltage");
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

    private SysIdRoutine makeVoltageSysIdRoutine(
            final Velocity<VoltageUnit> voltageRampRate,
            final Voltage stepVoltage,
            final Time timeout,
            final Consumer<Double> voltageConsumer
    ) {
        return new SysIdRoutine(
                new SysIdRoutine.Config(
                        voltageRampRate,
                        stepVoltage,
                        timeout,
                        state -> SignalLogger.writeString(String.format("%s-state", LogKey), state.toString())
                ),
                new SysIdRoutine.Mechanism(
                        voltageMeasure -> voltageConsumer.accept(voltageMeasure.in(Volts)),
                        null,
                        this
                )
        );
    }

    private SysIdRoutine makeTorqueCurrentSysIdRoutine(
            final Velocity<CurrentUnit> currentRampRate,
            final Current stepCurrent,
            final Time timeout,
            final Consumer<Double> torqueCurrentConsumer
    ) {
        return new SysIdRoutine(
                new SysIdRoutine.Config(
                        Volts.per(Second).of(currentRampRate.baseUnitMagnitude()),
                        Volts.of(stepCurrent.baseUnitMagnitude()),
                        timeout,
                        state -> SignalLogger.writeString(String.format("%s-state", LogKey), state.toString())
                ),
                new SysIdRoutine.Mechanism(
                        voltageMeasure -> torqueCurrentConsumer.accept(voltageMeasure.in(Volts)),
                        null,
                        this
                )
        );
    }

    private Command makeRollerSysIdCommand(final SysIdRoutine sysIdRoutine) {
        return Commands.sequence(
                sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward),
                Commands.waitSeconds(1),
                sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse),
                Commands.waitSeconds(1),
                sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward),
                Commands.waitSeconds(1),
                sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse)
        );
    }

    public Command coralVoltageSysIdCommand() {
        return makeRollerSysIdCommand(rollerVoltageSysIdRoutine);
    }

    public Command coralTorqueCurrentSysIdCommand() {
        return makeRollerSysIdCommand(rollerTorqueCurrentSysIdRoutine);
    }

}
