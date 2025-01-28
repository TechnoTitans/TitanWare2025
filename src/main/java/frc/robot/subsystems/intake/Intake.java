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

    public final Trigger isCoralIntaking = new Trigger(() -> coralRollerVelocitySetpoint > 0.0);
    public final Trigger isCoralOuttaking = new Trigger(() -> coralRollerVelocitySetpoint < 0.0);
    public final Trigger isCoralIntakeStopped = isCoralIntaking.negate().and(isCoralOuttaking.negate());
    public final Trigger isAlgaeIntaking = new Trigger(() -> algaeRollerVelocitySetpoint > 0.0);
    public final Trigger isAlgaeOuttaking = new Trigger(() -> algaeRollerVelocitySetpoint < 0.0);
    public final Trigger isAlgaeIntakeStopped = isAlgaeIntaking.negate().and(isAlgaeOuttaking.negate());

    public final Trigger isCoralPresent = new Trigger(this::isCoralPresent);
    public final Trigger isAlgaePresent = new Trigger(this::isAlgaePresent);

    public final DoubleSupplier coralDistance = this::getCoralDistance;
    public final LinearFilter algaeDetectionCurrentFilter = LinearFilter.movingAverage(16);

    public Intake(final Constants.RobotMode mode, final HardwareConstants.IntakeConstants constants) {
        this.intakeIO = switch (mode) {
            case REAL -> new IntakeIOReal(constants);
            case SIM -> new IntakeIOSim(constants);
            case REPLAY, DISABLED -> new IntakeIO() {};
        };

        this.inputs = new IntakeIOInputsAutoLogged();

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

        Logger.recordOutput(LogKey + "/CoralRollerVelocitySetpoint", coralRollerVelocitySetpoint);
        Logger.recordOutput(LogKey + "/AlgaeRollerVelocitySetpoint", algaeRollerVelocitySetpoint);

        Logger.recordOutput(
                LogKey + "/PeriodicIOPeriodMs",
                LogUtils.microsecondsToMilliseconds(RobotController.getFPGATime() - intakePeriodicUpdateStart)
        );
    }

    private boolean isCoralPresent() {
        return inputs.coralCANRangeIsDetected;
    }

    private boolean isAlgaePresent() {
        final double filteredCurrent = algaeDetectionCurrentFilter.calculate(inputs.algaeRollerTorqueCurrentAmps);
        return filteredCurrent >= 30;
    }

    private double getCoralDistance() {
        return inputs.coralCANRangeDistanceMeters;
    }

    public Command scoreCoral() {
        return runCoralRollerVelocity(-2)
                .onlyIf(isCoralPresent)
                .until(isCoralPresent.negate())
                .withTimeout(2)
                .andThen(coralInstantStopCommand());
    }

    public Command runCoralRollerVelocity(final double velocityRotsPerSec) {
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

    public Command runAlgaeRollerVelocity(final double velocityRotsPerSec) {
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

    public Command runCoralRollerVoltage(final double volts) {
        return runEnd(
                () -> intakeIO.toCoralRollerVoltage(volts),
                () -> intakeIO.toCoralRollerVoltage(0.0)
        );
    }

    public Command runAlgaeRollerVoltage(final double volts) {
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
