package frc.robot.subsystems.intake.ground;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.units.CurrentUnit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.Constants;
import frc.robot.constants.HardwareConstants;
import frc.robot.utils.logging.LogUtils;
import org.littletonrobotics.junction.Logger;

import java.util.function.Consumer;

import static edu.wpi.first.units.Units.*;

public class GroundIntake extends SubsystemBase {
    protected static final String LogKey = "GroundIntake";

    private final GroundIntakeIO groundIntakeIO;
    private final GroundIntakeIOInputsAutoLogged inputs;

    private final SysIdRoutine wheelVoltageSysIdRoutine;
    private final SysIdRoutine wheelTorqueCurrentSysIdRoutine;

    private boolean intaking = false;
    private boolean outtaking = false;

    private double wheelVelocitySetpoint = 0.0;
    private double wheelVoltageSetpoint = 0.0;
    private double wheelTorqueCurrentSetpoint = 0.0;

    private final EventLoop eventLoop;

    public final Trigger isIntaking;
    public final Trigger isOuttaking;
    public final Trigger isCoralPresent;

    public GroundIntake(final Constants.RobotMode mode, final HardwareConstants.GroundIntakeConstants constants) {
        this.groundIntakeIO = switch (mode) {
            case REAL -> new GroundIntakeIOReal(constants);
            case SIM -> new GroundIntakeIOSim(constants);
            case REPLAY, DISABLED -> new GroundIntakeIO() {};
        };

        this.inputs = new GroundIntakeIOInputsAutoLogged();

        this.eventLoop = new EventLoop();

        this.isIntaking = new Trigger(eventLoop, () -> intaking);
        this.isOuttaking = new Trigger(eventLoop, () -> outtaking);
        this.isCoralPresent = new Trigger(eventLoop, () -> inputs.coralDetected);

        this.wheelVoltageSysIdRoutine = makeVoltageSysIdRoutine(
                Volts.of(2).per(Second),
                Volts.of(10),
                Seconds.of(10),
                groundIntakeIO::toWheelVoltage
        );
        this.wheelTorqueCurrentSysIdRoutine = makeTorqueCurrentSysIdRoutine(
                Amps.of(4).per(Second),
                Amp.of(40),
                Seconds.of(10),
                groundIntakeIO::toWheelTorqueCurrent
        );

        this.groundIntakeIO.config();
    }

    @Override
    public void periodic() {
        final double groundIntakePeriodicUpdateStart = RobotController.getFPGATime();

        groundIntakeIO.updateInputs(inputs);
        Logger.processInputs(LogKey, inputs);

        eventLoop.poll();

        Logger.recordOutput(LogKey + "/WheelVelocitySetpoint", wheelVelocitySetpoint);
        Logger.recordOutput(LogKey + "/WheelVoltageSetpoint", wheelVoltageSetpoint);
        Logger.recordOutput(LogKey + "/WheelTorqueCurrentSetpoint", wheelTorqueCurrentSetpoint);

        Logger.recordOutput(LogKey + "/Trigger/IsIntaking", isIntaking);
        Logger.recordOutput(LogKey + "/Trigger/IsOuttaking", isOuttaking);

        Logger.recordOutput(
                LogKey + "/PeriodicIOPeriodMs",
                LogUtils.microsecondsToMilliseconds(RobotController.getFPGATime() - groundIntakePeriodicUpdateStart)
        );
    }

    public Command intake() {
        return Commands.sequence(
                runOnce(() -> this.intaking = true),
                toWheelVelocity(9)
        )
                .finallyDo(() -> this.intaking = false)
                .withName(LogKey + "-Intake");
    }

    public Command hold() {
        return toInstantWheelTorqueCurrent(11)
                .withName(LogKey + "-Holding");
    }

    public Command handOff() {
        return Commands.sequence(
                runOnce(() -> this.outtaking = true),
                toWheelVelocity(-15)
        )
                .finallyDo(() -> this.outtaking = false)
                .withName(LogKey + "-HandingOff");
    }

    private Command toInstantWheelVoltage(final double volts) {
        return runOnce(
                () -> {
                    this.wheelVoltageSetpoint = volts;
                    groundIntakeIO.toWheelVoltage(volts);
                }
        ).withName(LogKey + "-ToInstantWheelVoltage");
    }

    private Command toInstantWheelTorqueCurrent(final double torqueCurrentAmps) {
        return runOnce(
                () -> {
                    this.wheelTorqueCurrentSetpoint = torqueCurrentAmps;
                    groundIntakeIO.toWheelTorqueCurrent(torqueCurrentAmps);
                }
        ).withName(LogKey + "-ToInstantWheelTorqueCurrent");
    }

    private Command toWheelVelocity(final double velocityRotsPerSec) {
        return runEnd(
                () -> {
                    this.wheelVelocitySetpoint = velocityRotsPerSec;
                    groundIntakeIO.toWheelVelocity(wheelVelocitySetpoint);
                },
                () -> {
                    this.wheelVelocitySetpoint = 0.0;
                    groundIntakeIO.toWheelVelocity(wheelVelocitySetpoint);
                }
        ).withName(LogKey + "-ToWheelVelocity");
    }

    public Command instantStopCommand() {
        return Commands.runOnce(() -> {
                    this.intaking = false;
                    this.outtaking = false;
                    this.wheelVelocitySetpoint = 0.0;
                    this.wheelVoltageSetpoint = 0.0;
                    groundIntakeIO.toWheelVoltage(0);
                }
        ).withName(LogKey + "-InstantStop");
    }

    public void setCoralCANRangeDistance(final double distanceMeters) {
        groundIntakeIO.setCoralCANRangeDistance(distanceMeters);
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

    private Command makeWheelSysIdCommand(final SysIdRoutine sysIdRoutine) {
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
        return makeWheelSysIdCommand(wheelVoltageSysIdRoutine);
    }

    public Command coralTorqueCurrentSysIdCommand() {
        return makeWheelSysIdCommand(wheelTorqueCurrentSysIdRoutine);
    }
}
