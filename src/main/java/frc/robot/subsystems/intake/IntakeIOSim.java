package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.UpdateModeValue;
import com.ctre.phoenix6.sim.ChassisReference;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.constants.HardwareConstants;
import frc.robot.utils.MoreDCMotor;
import frc.robot.utils.closeables.ToClose;
import frc.robot.utils.control.DeltaTime;
import frc.robot.utils.sim.motors.TalonFXSim;

public class IntakeIOSim implements IntakeIO {
    private static final double SIM_UPDATE_PERIOD_SEC = 0.005;

    private final DeltaTime deltaTime;
    private final HardwareConstants.IntakeConstants constants;

    private final TalonFX coralRollerMotor;
    private final TalonFX algaeRollerMotor;
    private final CANrange coralCANRange;

    private final TalonFXSim coralRollerTalonFXSim;
    private final TalonFXSim algaeRollerTalonFXSim;

    private final VelocityTorqueCurrentFOC velocityTorqueCurrentFOC;
    private final TorqueCurrentFOC torqueCurrentFOC;
    private final VoltageOut voltageOut;

    private final StatusSignal<Angle> coralPosition;
    private final StatusSignal<AngularVelocity> coralVelocity;
    private final StatusSignal<Voltage> coralVoltage;
    private final StatusSignal<Current> coralTorqueCurrent;
    private final StatusSignal<Temperature> coralDeviceTemp;
    private final StatusSignal<Angle> algaePosition;
    private final StatusSignal<AngularVelocity> algaeVelocity;
    private final StatusSignal<Voltage> algaeVoltage;
    private final StatusSignal<Current> algaeTorqueCurrent;
    private final StatusSignal<Temperature> algaeDeviceTemp;
    private final StatusSignal<Distance> coralCANRangeDistance;
    private final StatusSignal<Boolean> coralCANRangeIsDetected;

    public IntakeIOSim(final HardwareConstants.IntakeConstants constants) {
        this.deltaTime = new DeltaTime(true);
        this.constants = constants;

        this.coralRollerMotor = new TalonFX(constants.coralRollerMotorID(), constants.CANBus());
        this.algaeRollerMotor = new TalonFX(constants.algaeRollerMotorID(), constants.CANBus());
        this.coralCANRange = new CANrange(constants.coralCANRangeId(), constants.CANBus());

        final DCMotorSim coralRollerMotorSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                    0.19557 / (2 * Math.PI),
                    2.9856 / (2 * Math.PI)
            ),
            MoreDCMotor.getKrakenX44(1)
        );
        this.coralRollerTalonFXSim = new TalonFXSim(
                coralRollerMotor,
                constants.coralGearing(),
                coralRollerMotorSim::update,
                coralRollerMotorSim::setInputVoltage,
                coralRollerMotorSim::getAngularPositionRad,
                coralRollerMotorSim::getAngularVelocityRadPerSec
        );

        final DCMotorSim algaeRollerMotorSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                    0.19557 / (2 * Math.PI),
                    2.9856 / (2 * Math.PI)
            ),
            MoreDCMotor.getKrakenX44(1)
        );
        this.algaeRollerTalonFXSim = new TalonFXSim(
                algaeRollerMotor,
                constants.algaeGearing(),
                algaeRollerMotorSim::update,
                algaeRollerMotorSim::setInputVoltage,
                algaeRollerMotorSim::getAngularPositionRad,
                algaeRollerMotorSim::getAngularVelocityRadPerSec
        );

        this.velocityTorqueCurrentFOC = new VelocityTorqueCurrentFOC(0);
        this.torqueCurrentFOC = new TorqueCurrentFOC(0);
        this.voltageOut = new VoltageOut(0);

        this.coralPosition = coralRollerMotor.getPosition();
        this.coralVelocity = coralRollerMotor.getVelocity();
        this.coralVoltage = coralRollerMotor.getMotorVoltage();
        this.coralTorqueCurrent = coralRollerMotor.getTorqueCurrent();
        this.coralDeviceTemp = coralRollerMotor.getDeviceTemp();
        this.algaePosition = algaeRollerMotor.getPosition();
        this.algaeVelocity = algaeRollerMotor.getVelocity();
        this.algaeVoltage = algaeRollerMotor.getMotorVoltage();
        this.algaeTorqueCurrent = algaeRollerMotor.getTorqueCurrent();
        this.algaeDeviceTemp = algaeRollerMotor.getDeviceTemp();
        this.coralCANRangeDistance = coralCANRange.getDistance();
        this.coralCANRangeIsDetected = coralCANRange.getIsDetected();

        final Notifier simUpdateNotifier = new Notifier(() -> {
        final double dt = deltaTime.get();
            coralRollerTalonFXSim.update(dt);
            algaeRollerTalonFXSim.update(dt);
        });
        ToClose.add(simUpdateNotifier);
        simUpdateNotifier.setName(String.format(
                "SimUpdate(%d, %d)",
                coralRollerMotor.getDeviceID(),
                algaeRollerMotor.getDeviceID()
        ));
        simUpdateNotifier.startPeriodic(SIM_UPDATE_PERIOD_SEC);
    }

    @Override
    public void config() {
        final InvertedValue coralRollerInvertedValue = InvertedValue.CounterClockwise_Positive;
        final TalonFXConfiguration coralConfiguration = new TalonFXConfiguration();
        coralConfiguration.Slot0 = new Slot0Configs()
                .withKS(3.3326)
                .withKV(0.15104)
                .withKA(0.2004)
                .withKP(10.746);
        coralConfiguration.TorqueCurrent.PeakForwardTorqueCurrent = 60;
        coralConfiguration.TorqueCurrent.PeakReverseTorqueCurrent = -60;
        coralConfiguration.CurrentLimits.StatorCurrentLimit = 60;
        coralConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        coralConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        coralConfiguration.MotorOutput.Inverted = coralRollerInvertedValue;
        coralConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        coralConfiguration.Feedback.SensorToMechanismRatio = constants.coralGearing();
        coralRollerMotor.getConfigurator().apply(coralConfiguration);

        final InvertedValue algaeRollerInvertedValue = InvertedValue.CounterClockwise_Positive;
        final TalonFXConfiguration algaeConfiguration = new TalonFXConfiguration();
        algaeConfiguration.Slot0 = new Slot0Configs()
                .withKS(3.3326)
                .withKV(0.15104)
                .withKA(0.2004)
                .withKP(10.746);
        algaeConfiguration.TorqueCurrent.PeakForwardTorqueCurrent = 60;
        algaeConfiguration.TorqueCurrent.PeakReverseTorqueCurrent = -60;
        algaeConfiguration.CurrentLimits.StatorCurrentLimit = 60;
        algaeConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        algaeConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        algaeConfiguration.MotorOutput.Inverted = algaeRollerInvertedValue;
        algaeConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        algaeConfiguration.Feedback.SensorToMechanismRatio = constants.algaeGearing();
        algaeRollerMotor.getConfigurator().apply(algaeConfiguration);

        final CANrangeConfiguration CANRangeConfiguration = new CANrangeConfiguration();
        CANRangeConfiguration.ToFParams.UpdateMode = UpdateModeValue.ShortRange100Hz;
        CANRangeConfiguration.ProximityParams.ProximityThreshold = Units.inchesToMeters(11);
        CANRangeConfiguration.ProximityParams.ProximityHysteresis = Units.inchesToMeters(1);
        coralCANRange.getConfigurator().apply(CANRangeConfiguration);

        BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                coralPosition,
                coralVelocity,
                coralVoltage,
                coralTorqueCurrent,
                algaePosition,
                algaeVelocity,
                algaeVoltage,
                algaeTorqueCurrent,
                coralCANRangeDistance,
                coralCANRangeIsDetected
        );

        BaseStatusSignal.setUpdateFrequencyForAll(
                4,
                coralDeviceTemp,
                algaeDeviceTemp
        );

        ParentDevice.optimizeBusUtilizationForAll(
                coralRollerMotor,
                algaeRollerMotor,
                coralCANRange
        );

        coralRollerMotor.getSimState().Orientation = ChassisReference.CounterClockwise_Positive;
        algaeRollerMotor.getSimState().Orientation = ChassisReference.CounterClockwise_Positive;
        coralCANRange.getSimState().setDistance(10);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                coralPosition,
                coralVelocity,
                coralVoltage,
                coralTorqueCurrent,
                coralDeviceTemp,
                algaePosition,
                algaeVelocity,
                algaeVoltage,
                algaeTorqueCurrent,
                algaeDeviceTemp,
                coralCANRangeDistance,
                coralCANRangeIsDetected
        );

        inputs.coralRollerPositionRots = coralPosition.getValueAsDouble();
        inputs.coralRollerVelocityRotsPerSec = coralVelocity.getValueAsDouble();
        inputs.coralRollerVoltage = coralVoltage.getValueAsDouble();
        inputs.coralRollerTorqueCurrentAmps = coralTorqueCurrent.getValueAsDouble();
        inputs.coralRollerTempCelsius = coralDeviceTemp.getValueAsDouble();
        inputs.algaeRollerPositionRots = algaePosition.getValueAsDouble();
        inputs.algaeRollerVelocityRotsPerSec = algaeVelocity.getValueAsDouble();
        inputs.algaeRollerVoltage = algaeVoltage.getValueAsDouble();
        inputs.algaeRollerTorqueCurrentAmps = algaeTorqueCurrent.getValueAsDouble();
        inputs.algaeRollerTempCelsius = algaeDeviceTemp.getValueAsDouble();
        inputs.coralCANRangeDistanceMeters = coralCANRangeDistance.getValueAsDouble();
    }

    @Override
    public void toCoralRollerVelocity(final double coralRollerVelocityRotsPerSec) {
        coralRollerMotor.setControl(velocityTorqueCurrentFOC.withVelocity(coralRollerVelocityRotsPerSec));
    }

    @Override
    public void toCoralRollerVoltage(final double volts) {
        coralRollerMotor.setControl(voltageOut.withOutput(volts));
    }

    @Override
    public void toCoralRollerTorqueCurrent(final double torqueCurrentAmps) {
        coralRollerMotor.setControl(torqueCurrentFOC.withOutput(torqueCurrentAmps));
    }

    @Override
    public void toAlgaeRollerVelocity(final double algaeRollerVelocityRotsPerSec) {
        algaeRollerMotor.setControl(velocityTorqueCurrentFOC.withVelocity(algaeRollerVelocityRotsPerSec));
    }

    @Override
    public void toAlgaeRollerVoltage(final double volts) {
        algaeRollerMotor.setControl(voltageOut.withOutput(volts));
    }

    @Override
    public void toAlgaeRollerTorqueCurrent(final double torqueCurrentAmps) {
        algaeRollerMotor.setControl(torqueCurrentFOC.withOutput(torqueCurrentAmps));
    }

    @Override
    public void setCANRangeDistance(final double gamepieceDistanceMeters) {
        coralCANRange.getSimState().setDistance(gamepieceDistanceMeters);
    }
}
