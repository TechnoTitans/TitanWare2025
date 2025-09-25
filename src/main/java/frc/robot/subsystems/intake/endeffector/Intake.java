package frc.robot.subsystems.intake.endeffector;

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
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.Constants;
import frc.robot.constants.HardwareConstants;
import frc.robot.utils.commands.LoggedTrigger;
import frc.robot.utils.logging.LogUtils;
import org.littletonrobotics.junction.Logger;

import java.util.Map;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;

public class Intake extends SubsystemBase {
    private static final InterpolatingDoubleTreeMap coralTreeMap = new InterpolatingDoubleTreeMap();

    static {
        //raw, actual
        coralTreeMap.put(0.0, 0.0);
        coralTreeMap.put(0.021, Units.inchesToMeters(0.6));
        coralTreeMap.put(0.055, Units.inchesToMeters(1.3));
        coralTreeMap.put(0.075, Units.inchesToMeters(1.75));
        coralTreeMap.put(0.094, Units.inchesToMeters(2.45));
        coralTreeMap.put(0.1, Units.inchesToMeters(2.5));
        coralTreeMap.put(0.11, Units.inchesToMeters(3.0));
        coralTreeMap.put(0.133, Units.inchesToMeters(3.7));
        coralTreeMap.put(0.155, Units.inchesToMeters(4.6));
        coralTreeMap.put(0.165, Units.inchesToMeters(5.0));
        coralTreeMap.put(0.178, Units.inchesToMeters(5.5));
        coralTreeMap.put(0.181, Units.inchesToMeters(5.75));
        coralTreeMap.put(0.197, Units.inchesToMeters(6.25));
        coralTreeMap.put(0.207, Units.inchesToMeters(6.6));
        coralTreeMap.put(0.229, Units.inchesToMeters(7.25));
        coralTreeMap.put(0.24, Units.inchesToMeters(7.75));
        coralTreeMap.put(0.246, Units.inchesToMeters(8.1));
        coralTreeMap.put(0.268, Units.inchesToMeters(8.75));
        coralTreeMap.put(0.272, Units.inchesToMeters(9.0));
        coralTreeMap.put(0.28, Units.inchesToMeters(9.25));
        coralTreeMap.put(0.295, Units.inchesToMeters(9.75));
        coralTreeMap.put(0.301, Units.inchesToMeters(10.2));
        coralTreeMap.put(0.321, Units.inchesToMeters(11.0));
        coralTreeMap.put(0.42, Units.inchesToMeters(15.5));
    }

    public enum ScoreMode {
        RUN_UNTIL_NO_CORAL,
        RUN_FOR_TIME
    }

    protected static final String LogKey = "Intake";
    private static final double CoralRadiusFromODMeters = Units.inchesToMeters(4.5 / 2);
    private static final double CoralIntakeCenterDistanceMeters = Units.inchesToMeters(15.5 / 2);
    private static final double NoCoralTOFReading = 0.36;
    private static final double AlgaeDetectedCurrent = 25;

    private final IntakeIO intakeIO;
    private final IntakeIOInputsAutoLogged inputs;

    private final SysIdRoutine rollerVoltageSysIdRoutine;
    private final SysIdRoutine rollerTorqueCurrentSysIdRoutine;

    private boolean coralIntaking = false;
    private boolean coralOuttaking = false;
    private boolean algaeIntaking = false;
    private boolean algaeOuttaking = false;
    private boolean coralHandoff = false;

    private double rollerVelocitySetpoint = 0.0;
    private double rollerVoltageSetpoint = 0.0;
    private double rollerTorqueCurrentSetpoint = 0.0;

    private final EventLoop eventLoop;
    public final LoggedTrigger.Group group;

    public final LoggedTrigger isCoralIntaking;
    public final LoggedTrigger isCoralOuttaking;
    public final LoggedTrigger isCoralIntakeStopped;
    public final LoggedTrigger isAlgaeIntaking;
    public final LoggedTrigger isAlgaeOuttaking;
    public final LoggedTrigger isAlgaeIntakeStopped;
    public final LoggedTrigger isCoralHandingOff;

    public final LoggedTrigger isCoralPresent;
    public final LoggedTrigger isCurrentAboveAlgaeThreshold;

    public final DoubleSupplier coralDistanceMeters = this::getCoralDistanceMeters;
    public final LinearFilter coralDistanceFilter = LinearFilter.movingAverage(25);
    public final DoubleSupplier coralDistanceIntakeCenterMeters = this::getCoralDistanceFromCenterIntakeMeters;
    public final LinearFilter currentFilter = LinearFilter.movingAverage(16);

    public Intake(final Constants.RobotMode mode, final HardwareConstants.IntakeConstants constants) {
        this.intakeIO = switch (mode) {
            case REAL -> new IntakeIOReal(constants);
            case SIM -> new IntakeIOSim(constants);
            case REPLAY, DISABLED -> new IntakeIO() {
            };
        };

        this.inputs = new IntakeIOInputsAutoLogged();

        this.eventLoop = new EventLoop();
        this.group = LoggedTrigger.Group.from(LogKey, eventLoop);

        this.isCoralIntaking = group.t("isCoralIntaking", () -> coralIntaking);
        this.isCoralOuttaking = group.t("isCoralOuttaking", () -> coralOuttaking);
        this.isCoralIntakeStopped = isCoralIntaking.negate().and(isCoralOuttaking.negate());

        this.isAlgaeIntaking = group.t("isAlgaeIntaking", () -> algaeIntaking);
        this.isAlgaeOuttaking = group.t("isAlgaeOuttaking", () -> algaeOuttaking);
        this.isAlgaeIntakeStopped = isAlgaeIntaking.negate().and(isAlgaeOuttaking.negate());

        this.isCoralHandingOff = group.t("isCoralHandingOff", () -> coralHandoff);

        this.isCoralPresent = group.t("isCoralPresent", this::isCoralPresent).debounce(0.25);
        this.isCurrentAboveAlgaeThreshold = group.t(
                "isCurrentAboveAlgaeThreshold",
                this::isCurrentAboveAboveAlgaeThreshold
        ).debounce(0.25);

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

        Logger.recordOutput(LogKey + "/RollerVelocitySetpoint", rollerVelocitySetpoint);
        Logger.recordOutput(LogKey + "/RollerVoltageSetpoint", rollerVoltageSetpoint);
        Logger.recordOutput(LogKey + "/RollerTorqueCurrentSetpoint", rollerTorqueCurrentSetpoint);

        Logger.recordOutput(LogKey + "/LoggedTrigger/IsCoralPresent", isCoralPresent);
        Logger.recordOutput(LogKey + "/LoggedTrigger/IsCoralOuttaking", isCoralOuttaking);
        Logger.recordOutput(LogKey + "/LoggedTrigger/IsCoralIntakeStopped", isCoralIntakeStopped);

        Logger.recordOutput(LogKey + "/LoggedTrigger/IsCurrentAboveAlgaeThreshold", isCurrentAboveAlgaeThreshold);
        Logger.recordOutput(LogKey + "/LoggedTrigger/IsAlgaeOuttaking", isAlgaeOuttaking);
        Logger.recordOutput(LogKey + "/LoggedTrigger/IsAlgaeIntakeStopped", isAlgaeIntakeStopped);

        Logger.recordOutput(LogKey + "/LoggedTrigger/IsCoralHandingOff", isCoralHandingOff);

        Logger.recordOutput(LogKey + "/FilteredCoralDistanceMeters", getFilteredCoralDistanceMeters());
        Logger.recordOutput(LogKey + "/TreemapCoralDistanceMeters", getCoralDistanceMeters());
        Logger.recordOutput(LogKey + "/CoralDistanceFromCenterMeters", getCoralDistanceFromCenterIntakeMeters());

        Logger.recordOutput(LogKey + "/FilteredCurrentAmps", getFilteredCurrent());

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
        return getCoralDistanceMeters() < NoCoralTOFReading;
    }

    private double getFilteredCurrent() {
        return currentFilter.calculate(inputs.rollerTorqueCurrentAmps);
    }

    private boolean isCurrentAboveAboveAlgaeThreshold() {
        return Math.abs(getFilteredCurrent()) >= AlgaeDetectedCurrent;
    }

    public Command intakeCoralHP() {
        return Commands.sequence(
                runOnce(() -> this.coralIntaking = true),
                toRollerVelocity(9)
        )
                .finallyDo(() -> this.coralIntaking = false)
                .withName(LogKey + "-IntakeCoralHP");
    }

    public Command handOff() {
        return Commands.sequence(
                runOnce(() -> this.coralHandoff = true),
                toRollerVelocity(10)
        )
                .finallyDo(() -> this.coralHandoff = false)
                .withName(LogKey + "-Handoff");
    }


    public Command holdCoral() {
        return toInstantRollerTorqueCurrent(11)
                .withName(LogKey + "-HoldCoral");
    }

    public Command holdAlgae() {
        return toInstantRollerTorqueCurrent(-40)
                .withName(LogKey + "-HoldAlgae");
    }

    public Command scoreCoral(final Supplier<ScoreMode> scoreModeSupplier) {
        return Commands.sequence(
                        runOnce(() -> this.coralOuttaking = true),
                        Commands.select(
                                Map.of(
                                    ScoreMode.RUN_UNTIL_NO_CORAL,
                                        Commands.sequence(
                                                toInstantRollerVoltage(-9),
                                                Commands.waitUntil(isCoralPresent.negate())
                                                        .withTimeout(2)
                                        ),
                                    ScoreMode.RUN_FOR_TIME,
                                        Commands.sequence(
                                                toInstantRollerVoltage(-8),
                                                Commands.waitSeconds(0.2)
                                        )
                                ),
                                scoreModeSupplier
                        ),
                        instantStopCommand()
                )
                .finallyDo(() -> this.coralOuttaking = false)
                .withName(LogKey + "-ScoreCoral");
    }


    public Command ejectCoral() {
        return Commands.sequence(
                runOnce(() -> this.coralOuttaking = true),
                toInstantRollerVoltage(-9)
        )
                .finallyDo(() -> this.coralOuttaking = false)
                .withName(LogKey + "-EjectCoral");
    }

    public Command intakeAlgae() {
        return Commands.sequence(
                runOnce(() -> this.algaeIntaking = true),
                toRollerVelocity(-10)
        )
                .finallyDo(() -> this.algaeIntaking = false)
                .withName(LogKey + "-IntakeAlgae");
    }

    public Command scoreAlgae() {
        return Commands.sequence(
                runOnce(() -> this.algaeOuttaking = true),
                toInstantRollerVoltage(9),
                Commands.waitUntil(isCurrentAboveAlgaeThreshold.negate())
                        .withTimeout(1),
                Commands.waitSeconds(0.1),
                instantStopCommand()
        )
                .finallyDo(() -> this.algaeOuttaking = false)
                .withName(LogKey + "-ScoreAlgae");
    }

    public Command netAlgae() {
        return Commands.sequence(
                        runOnce(() -> this.algaeOuttaking = true),
                        toInstantRollerVoltage(8),
                        Commands.waitSeconds(0.23),
                        instantStopCommand()
                )
                .finallyDo(() -> this.algaeOuttaking = false)
                .withName(LogKey + "-NetAlgae");
    }


    private Command toInstantRollerVoltage(final double volts) {
        return runOnce(
                () -> {
                    this.rollerVoltageSetpoint = volts;
                    intakeIO.toRollerVoltage(volts);
                }
        ).withName(LogKey + "-ToInstantRollerVoltage");
    }

    private Command toInstantRollerTorqueCurrent(final double torqueCurrentAmps) {
        return runOnce(
                () -> {
                    this.rollerTorqueCurrentSetpoint = torqueCurrentAmps;
                    intakeIO.toRollerTorqueCurrent(torqueCurrentAmps);
                }
        ).withName(LogKey + "-ToInstantRollerTorqueCurrent");
    }

    @SuppressWarnings("SameParameterValue")
    private Command toRollerVelocity(final double velocityRotsPerSec) {
        return runEnd(
                () -> {
                    this.rollerVelocitySetpoint = velocityRotsPerSec;
                    intakeIO.toRollerVelocity(rollerVelocitySetpoint);
                },
                () -> {
                    this.rollerVelocitySetpoint = 0.0;
                    intakeIO.toRollerVelocity(rollerVelocitySetpoint);
                }
        ).withName(LogKey + "-ToRollerVelocity");
    }

    public Command instantStopCommand() {
        return Commands.runOnce(() -> {
                    this.coralIntaking = false;
                    this.coralOuttaking = false;
                    this.rollerVelocitySetpoint = 0.0;
                    this.rollerVoltageSetpoint = 0.0;
                    intakeIO.toRollerVoltage(0);
                }
        ).withName(LogKey + "-InstantStop");
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

    @SuppressWarnings("unused")
    public Command coralVoltageSysIdCommand() {
        return makeRollerSysIdCommand(rollerVoltageSysIdRoutine);
    }

    @SuppressWarnings("unused")
    public Command coralTorqueCurrentSysIdCommand() {
        return makeRollerSysIdCommand(rollerTorqueCurrentSysIdRoutine);
    }
}