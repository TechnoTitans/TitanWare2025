package frc.robot.subsystems.intake;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.filter.LinearFilter;
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
import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.*;

public class Intake extends SubsystemBase {
    protected static final String LogKey = "Intake";

    private final IntakeIO intakeIO;
    private final IntakeIOInputsAutoLogged inputs;

    private final SysIdRoutine coralRollerVoltageSysIdRoutine;
    private final SysIdRoutine coralRollerTorqueCurrentSysIdRoutine;
    private final SysIdRoutine algaeRollerVoltageSysIdRoutine;
    private final SysIdRoutine algaeRollerTorqueCurrentSysIdRoutine;

    private double coralRollerVelocitySetpoint = 0.0;
    private double algaeRollerVelocitySetpoint = 0.0;

    private final EventLoop eventLoop;

    public final Trigger isCoralIntaking;
    public final Trigger isCoralOuttaking;
    public final Trigger isCoralIntakeStopped;
    public final Trigger isAlgaeIntaking;
    public final Trigger isAlgaeOuttaking;
    public final Trigger isAlgaeIntakeStopped;

    public final Trigger isCoralPresent;
    public final Trigger isAlgaePresent;

    public final DoubleSupplier coralDistanceMeters = this::getCoralDistanceMeters;
    public final LinearFilter algaeDetectionCurrentFilter = LinearFilter.movingAverage(16);

    public Intake(final Constants.RobotMode mode, final HardwareConstants.IntakeConstants constants) {
        this.intakeIO = switch (mode) {
            case REAL -> new IntakeIOReal(constants);
            case SIM -> new IntakeIOSim(constants);
            case REPLAY, DISABLED -> new IntakeIO() {};
        };

        this.inputs = new IntakeIOInputsAutoLogged();
        this.eventLoop = new EventLoop();

        this.isCoralIntaking = new Trigger(eventLoop, () -> coralRollerVelocitySetpoint > 0.0);
        this.isCoralOuttaking = new Trigger(eventLoop, () -> coralRollerVelocitySetpoint < 0.0);
        this.isCoralIntakeStopped = isCoralIntaking.negate().and(isCoralOuttaking.negate());

        this.isAlgaeIntaking = new Trigger(eventLoop, () -> algaeRollerVelocitySetpoint > 0.0);
        this.isAlgaeOuttaking = new Trigger(eventLoop, () -> algaeRollerVelocitySetpoint < 0.0);
        this.isAlgaeIntakeStopped = isAlgaeIntaking.negate().and(isAlgaeOuttaking.negate());

        this.isCoralPresent = new Trigger(eventLoop, this::isCoralPresent);
        this.isAlgaePresent = new Trigger(eventLoop, this::isAlgaePresent);

        this.coralRollerVoltageSysIdRoutine = makeVoltageSysIdRoutine(
                Volts.of(2).per(Second),
                Volts.of(10),
                Seconds.of(10),
                intakeIO::toCoralRollerVoltage
        );
        this.coralRollerTorqueCurrentSysIdRoutine = makeTorqueCurrentSysIdRoutine(
                Amps.of(4).per(Second),
                Amp.of(40),
                Seconds.of(10),
                intakeIO::toCoralRollerTorqueCurrent
        );

        this.algaeRollerVoltageSysIdRoutine = makeVoltageSysIdRoutine(
                Volts.of(2).per(Second),
                Volts.of(10),
                Seconds.of(10),
                intakeIO::toAlgaeRollerVoltage
        );
        this.algaeRollerTorqueCurrentSysIdRoutine = makeTorqueCurrentSysIdRoutine(
                Amps.of(4).per(Second),
                Amp.of(40),
                Seconds.of(10),
                intakeIO::toAlgaeRollerTorqueCurrent
        );

        this.intakeIO.config();
    }

    @Override
    public void periodic() {
        final double intakePeriodicUpdateStart = RobotController.getFPGATime();

        intakeIO.updateInputs(inputs);
        Logger.processInputs(LogKey, inputs);

        eventLoop.poll();

        Logger.recordOutput(LogKey + "/CoralRollerVelocitySetpoint", coralRollerVelocitySetpoint);
        Logger.recordOutput(LogKey + "/AlgaeRollerVelocitySetpoint", algaeRollerVelocitySetpoint);

        Logger.recordOutput(LogKey + "/isCoralPresent", isCoralPresent);
        Logger.recordOutput(LogKey + "/FilteredAlgae", getFilteredAlgaeCurrent());
        Logger.recordOutput(LogKey + "/isAlgaePresent", isAlgaePresent);

        Logger.recordOutput(
                LogKey + "/PeriodicIOPeriodMs",
                LogUtils.microsecondsToMilliseconds(RobotController.getFPGATime() - intakePeriodicUpdateStart)
        );
    }

    private double getCoralDistanceMeters() {
        return inputs.coralCANRangeDistanceMeters;
    }

    private boolean isCoralPresent() {
        return inputs.coralCANRangeIsDetected;
    }

    private double getFilteredAlgaeCurrent() {
        return algaeDetectionCurrentFilter.calculate(inputs.algaeRollerTorqueCurrentAmps);
    }

    private boolean isAlgaePresent() {
        return getFilteredAlgaeCurrent() >= 30;
    }

    public Command intakeCoralHP() {
        return toCoralRollerVelocity(3);
    }

    public Command scoreCoral() {
        return toCoralRollerVelocity(-2)
                .onlyIf(isCoralPresent)
                .until(isCoralPresent.negate())
                .withTimeout(2)
                .andThen(coralInstantStopCommand());
    }

    public Command intakeAlgae() {
        return toAlgaeRollerVelocity(3);
    }

    public Command scoreAlgae() {
        return toAlgaeRollerVelocity(-2)
                .onlyIf(isAlgaePresent)
                .until(isAlgaePresent.negate())
                .withTimeout(2)
                .andThen(algaeInstantStopCommand());
    }

    public Command toCoralRollerVelocity(final double velocityRotsPerSec) {
        return runEnd(
                () -> {
                    coralRollerVelocitySetpoint = velocityRotsPerSec;
                    intakeIO.toCoralRollerVelocity(coralRollerVelocitySetpoint);
                },
                () -> {
                    coralRollerVelocitySetpoint = 0.0;
                    intakeIO.toCoralRollerVelocity(coralRollerVelocitySetpoint);
                }
        );
    }

    public Command toAlgaeRollerVelocity(final double velocityRotsPerSec) {
        return runEnd(
                () -> {
                    algaeRollerVelocitySetpoint = velocityRotsPerSec;
                    intakeIO.toAlgaeRollerVelocity(velocityRotsPerSec);
                },
                () -> {
                    algaeRollerVelocitySetpoint = 0.0;
                    intakeIO.toAlgaeRollerVelocity(0.0);
                }
        );
    }

    public Command toCoralRollerVoltage(final double volts) {
        return runEnd(
                () -> intakeIO.toCoralRollerVoltage(volts),
                () -> intakeIO.toCoralRollerVoltage(0.0)
        );
    }

    public Command toAlgaeRollerVoltage(final double volts) {
        return runEnd(
                () -> intakeIO.toAlgaeRollerVoltage(volts),
                () -> intakeIO.toAlgaeRollerVoltage(0.0)
        );
    }

    public Command coralInstantStopCommand() {
        return Commands.runOnce(() -> {
                    this.coralRollerVelocitySetpoint = 0.0;
                    intakeIO.toCoralRollerVoltage(0);
                }
        );
    }

    public Command algaeInstantStopCommand() {
        return Commands.runOnce(() -> {
                    this.algaeRollerVelocitySetpoint = 0.0;
                    intakeIO.toAlgaeRollerVoltage(0);
                }
        );
    }

    public void setCANRangeDistance(final double gamepieceDistanceMeters) {
        intakeIO.setCANRangeDistance(gamepieceDistanceMeters);
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
        return makeRollerSysIdCommand(coralRollerVoltageSysIdRoutine);
    }
    public Command coralTorqueCurrentSysIdCommand() {
        return makeRollerSysIdCommand(coralRollerTorqueCurrentSysIdRoutine);
    }

    public Command algaeVoltageSysIdCommand() {
        return makeRollerSysIdCommand(algaeRollerVoltageSysIdRoutine);
    }
    public Command algaeTorqueCurrentSysIdCommand() {
        return makeRollerSysIdCommand(algaeRollerTorqueCurrentSysIdRoutine);
    }
}
