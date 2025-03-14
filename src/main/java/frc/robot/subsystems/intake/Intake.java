package frc.robot.subsystems.intake;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
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
    private static final InterpolatingDoubleTreeMap coralTreeMap = new InterpolatingDoubleTreeMap();

    static {
        //measure, actual
        coralTreeMap.put(0.0, 0.0);
        coralTreeMap.put(0.021, Units.inchesToMeters(0.6));
        coralTreeMap.put(0.055, Units.inchesToMeters(1.3));
        coralTreeMap.put(0.075, Units.inchesToMeters(1.75));
        coralTreeMap.put(0.094, Units.inchesToMeters(2.45));
        coralTreeMap.put(0.1, Units.inchesToMeters(2.75));
        coralTreeMap.put(0.11, Units.inchesToMeters(3.0));
        coralTreeMap.put(0.133, Units.inchesToMeters(3.7));
        coralTreeMap.put(0.155, Units.inchesToMeters(4.6));
        coralTreeMap.put(0.165, Units.inchesToMeters(5.0));
        coralTreeMap.put(0.18, Units.inchesToMeters(5.75));
        coralTreeMap.put(0.197, Units.inchesToMeters(6.25));
        coralTreeMap.put(0.207, Units.inchesToMeters(6.6));
        coralTreeMap.put(0.23, Units.inchesToMeters(7.5));
        coralTreeMap.put(0.237, Units.inchesToMeters(7.75));
        coralTreeMap.put(0.246, Units.inchesToMeters(8.1));
        coralTreeMap.put(0.272, Units.inchesToMeters(9.0));
        coralTreeMap.put(0.28, Units.inchesToMeters(9.25));
        coralTreeMap.put(0.301, Units.inchesToMeters(10.2));
        coralTreeMap.put(0.321, Units.inchesToMeters(11.0));
        coralTreeMap.put(0.42, Units.inchesToMeters(15.5));
    }

    protected static final String LogKey = "Intake";
    private static final double CoralRadiusFromODMeters = Units.inchesToMeters(4.5 / 2);
    private static final double CoralIntakeCenterDistanceMeters = Units.inchesToMeters(15.5 / 2);
    private static final double NoCoralTOFReading = 0.4;
    private static final double AlgaeDetectedCurrent = 25;

    private final IntakeIO intakeIO;
    private final IntakeIOInputsAutoLogged inputs;

    private final SysIdRoutine rollerVoltageSysIdRoutine;
    private final SysIdRoutine rollerTorqueCurrentSysIdRoutine;

    private boolean coralIntaking = false;
    private boolean coralOuttaking = false;
    private boolean algaeIntaking = false;
    private boolean algaeOuttaking = false;

    private double coralRollerVelocitySetpoint = 0.0;
    private double algaeRollerVelocitySetpoint = 0.0;

    private double coralRollerVoltageSetpoint = 0.0;
    private double algaeRollerVoltageSetpoint = 0.0;

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
    public final LinearFilter coralDistanceFilter = LinearFilter.movingAverage(25);
    public final DoubleSupplier coralDistanceIntakeCenterMeters = this::getCoralDistanceFromCenterIntakeMeters;
    public final LinearFilter algaeDetectionCurrentFilter = LinearFilter.movingAverage(16);

    public Intake(final Constants.RobotMode mode, final HardwareConstants.IntakeConstants constants) {
        this.intakeIO = switch (mode) {
            case REAL -> new IntakeIOReal(constants);
            case SIM -> new IntakeIOSim(constants);
            case REPLAY, DISABLED -> new IntakeIO() {
            };
        };

        this.inputs = new IntakeIOInputsAutoLogged();

        this.eventLoop = new EventLoop();

        this.isCoralIntaking = new Trigger(eventLoop, () -> coralIntaking);
        this.isCoralOuttaking = new Trigger(eventLoop, () -> coralOuttaking);
        this.isCoralIntakeStopped = isCoralIntaking.negate().and(isCoralOuttaking.negate());

        this.isAlgaeIntaking = new Trigger(eventLoop, () -> algaeIntaking);
        this.isAlgaeOuttaking = new Trigger(eventLoop, () -> algaeOuttaking);
        this.isAlgaeIntakeStopped = isAlgaeIntaking.negate().and(isAlgaeOuttaking.negate());

        this.isCoralPresent = new Trigger(eventLoop, this::isCoralPresent).debounce(0.5);
        this.isAlgaePresent = new Trigger(eventLoop, this::isAlgaePresent);

        this.rollerVoltageSysIdRoutine = makeVoltageSysIdRoutine(
                Volts.of(2).per(Second),
                Volts.of(10),
                Seconds.of(10),
                intakeIO::toRollerVoltage
        );
        this.rollerTorqueCurrentSysIdRoutine = makeTorqueCurrentSysIdRoutine(
                Amps.of(4).per(Second),
                Amp.of(40),
                Seconds.of(10),
                intakeIO::toRollerTorqueCurrent
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

        Logger.recordOutput(LogKey + "/CoralRollerVoltageSetpoint", coralRollerVoltageSetpoint);
        Logger.recordOutput(LogKey + "/AlgaeRollerVoltageSetpoint", algaeRollerVoltageSetpoint);

        Logger.recordOutput(LogKey + "/Trigger/IsCoralPresent", isCoralPresent);
        Logger.recordOutput(LogKey + "/Trigger/IsCoralOuttaking", isCoralOuttaking);
        Logger.recordOutput(LogKey + "/Trigger/IsCoralIntakeStopped", isCoralIntakeStopped);

        Logger.recordOutput(LogKey + "/Trigger/IsAlgaePresent", isAlgaePresent);
        Logger.recordOutput(LogKey + "/Trigger/IsAlgaeOuttaking", isAlgaeOuttaking);
        Logger.recordOutput(LogKey + "/Trigger/IsAlgaeIntakeStopped", isAlgaeIntakeStopped);
        Logger.recordOutput(LogKey + "/FilteredAlgae", getFilteredAlgaeCurrent());

        Logger.recordOutput(LogKey + "/FilteredCoralDistanceMeters", getFilteredCoralDistanceMeters());
        Logger.recordOutput(LogKey + "/OffsetCoralDistanceMeters", getCoralDistanceMeters());
        Logger.recordOutput(LogKey + "/CoralDistanceFromCenterIntakeMeters", getCoralDistanceFromCenterIntakeMeters());

        Logger.recordOutput(
                LogKey + "/PeriodicIOPeriodMs",
                LogUtils.microsecondsToMilliseconds(RobotController.getFPGATime() - intakePeriodicUpdateStart)
        );
    }

    private double getFilteredCoralDistanceMeters() {
        return coralDistanceFilter.calculate(inputs.coralTOFDistanceMeters);
    }

    private double getCoralDistanceMeters() {
        return coralTreeMap.get(getFilteredCoralDistanceMeters());
    }

    private double getCoralDistanceFromCenterIntakeMeters() {
        return getCoralDistanceMeters() + CoralRadiusFromODMeters - CoralIntakeCenterDistanceMeters;
    }

    private boolean isCoralPresent() {
        return getFilteredCoralDistanceMeters() < NoCoralTOFReading;
    }

    private double getFilteredAlgaeCurrent() {
        return algaeDetectionCurrentFilter.calculate(inputs.rollerTorqueCurrentAmps);
    }

    private boolean isAlgaePresent() {
        return Math.abs(getFilteredAlgaeCurrent()) >= AlgaeDetectedCurrent;
    }

    public Command intakeCoralHP() {
        return Commands.sequence(
                runOnce(() -> this.coralIntaking = true),
                toCoralRollerVelocity(9)
        ).finallyDo(() -> this.coralIntaking = false).withName("IntakeCoralHP");
    }

    public Command holdCoral() {
        return toInstantCoralRollerVoltage(0.8).withName("HoldCoral");
    }

    public Command holdAlgae() {
        return toInstantAlgaeRollerVoltage(-3).withName("HoldAlgae");
    }

    public Command scoreCoral() {
        return Commands.sequence(
                runOnce(() -> this.coralOuttaking = true),
                toInstantCoralRollerVoltage(-9),
                Commands.waitUntil(isCoralPresent.negate())
                        .withTimeout(2),
                Commands.waitSeconds(0.2),
                coralInstantStopCommand()
        ).finallyDo(() -> this.coralOuttaking = false).withName("ScoreCoral");
    }

    public Command intakeAlgae() {
        return Commands.sequence(
                runOnce(() -> this.algaeIntaking = true),
                toAlgaeRollerVelocity(-10)
        ).finallyDo(() -> this.algaeIntaking = false).withName("IntakeAlgae");
    }

    public Command scoreAlgae() {
        return Commands.sequence(
                runOnce(() -> this.algaeOuttaking = true),
                toInstantAlgaeRollerVoltage(-9),
                Commands.waitUntil(isAlgaePresent.negate()).withTimeout(1),
                Commands.waitSeconds(0.1),
                algaeInstantStopCommand()
        ).finallyDo(() -> this.algaeOuttaking = false).withName("ScoreAlgae");
    }

    private Command toInstantCoralRollerVoltage(final double volts) {
        return runOnce(
                () -> {
                    coralRollerVoltageSetpoint = volts;
                    intakeIO.toRollerVoltage(volts);
                }
        ).withName("ToInstantCoralRollerVoltage");
    }

    private Command toInstantAlgaeRollerVoltage(final double volts) {
        return runOnce(
                () -> {
                    algaeRollerVoltageSetpoint = volts;
                    intakeIO.toRollerVoltage(volts);
                }
        ).withName("ToInstantCoralRollerVoltage");
    }

    @SuppressWarnings("SameParameterValue")
    private Command toCoralRollerVelocity(final double velocityRotsPerSec) {
        return runEnd(
                () -> {
                    coralRollerVelocitySetpoint = velocityRotsPerSec;
                    intakeIO.toRollerVelocity(coralRollerVelocitySetpoint);
                },
                () -> {
                    coralRollerVelocitySetpoint = 0.0;
                    intakeIO.toRollerVelocity(coralRollerVelocitySetpoint);
                }
        ).withName("ToCoralRollerVelocity");
    }

    @SuppressWarnings("SameParameterValue")
    private Command toAlgaeRollerVelocity(final double velocityRotsPerSec) {
        return runEnd(
                () -> {
                    algaeRollerVelocitySetpoint = velocityRotsPerSec;
                    intakeIO.toRollerVelocity(velocityRotsPerSec);
                },
                () -> {
                    algaeRollerVelocitySetpoint = 0.0;
                    intakeIO.toRollerVelocity(0.0);
                }
        ).withName("ToAlgaeRollerVelocity");
    }

    public Command coralInstantStopCommand() {
        return Commands.runOnce(() -> {
                    this.coralIntaking = false;
                    this.coralOuttaking = false;
                    this.coralRollerVelocitySetpoint = 0.0;
                    this.coralRollerVoltageSetpoint = 0.0;
                    intakeIO.toRollerVoltage(0);
                }
        ).withName("CoralInstantStop");
    }

    public Command algaeInstantStopCommand() {
        return Commands.runOnce(() -> {
                    this.algaeIntaking = false;
                    this.algaeOuttaking = false;
                    this.algaeRollerVelocitySetpoint = 0.0;
                    this.algaeRollerVoltageSetpoint = 0.0;
                    intakeIO.toRollerVoltage(0);
                }
        ).withName("AlgaeInstantStop");
    }

    public void setTOFDistance(final double distanceMeters) {
        intakeIO.setTOFDistance(distanceMeters);
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