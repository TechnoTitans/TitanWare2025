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
import edu.wpi.first.units.measure.*;
import frc.robot.constants.HardwareConstants;

public class IntakeIOReal implements IntakeIO {
    private final HardwareConstants.IntakeConstants constants;

    private final TalonFX coralRollerMotor;
    private final TalonFX algaeRollerMotor;
    private final CANrange coralCANRange;

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

    public IntakeIOReal(final HardwareConstants.IntakeConstants constants) {
        this.constants = constants;

        this.coralRollerMotor = new TalonFX(constants.coralRollerMotorID(), constants.CANBus());
        this.algaeRollerMotor = new TalonFX(constants.algaeRollerMotorID(), constants.CANBus());
        this.coralCANRange = new CANrange(constants.coralCANRangeId(), constants.CANBus());

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
    }

    @Override
    public void config() {
        final TalonFXConfiguration coralConfiguration = new TalonFXConfiguration();
        coralConfiguration.Slot0 = new Slot0Configs()
//                .withKS(3.3326)
//                .withKV(0.15104)
//                .withKA(0.2004)
                .withKP(10.746);
        coralConfiguration.TorqueCurrent.PeakForwardTorqueCurrent = 60;
        coralConfiguration.TorqueCurrent.PeakReverseTorqueCurrent = -60;
        coralConfiguration.CurrentLimits.StatorCurrentLimit = 60;
        coralConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        coralConfiguration.CurrentLimits.SupplyCurrentLimit = 40;
        coralConfiguration.CurrentLimits.SupplyCurrentLowerLimit = 40;
        coralConfiguration.CurrentLimits.SupplyCurrentLowerTime = 1;
        coralConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
        coralConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        coralConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        coralConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        coralConfiguration.Feedback.SensorToMechanismRatio = constants.coralGearing();
        coralRollerMotor.getConfigurator().apply(coralConfiguration);

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
        algaeConfiguration.CurrentLimits.SupplyCurrentLimit = 40;
        algaeConfiguration.CurrentLimits.SupplyCurrentLowerLimit = 40;
        algaeConfiguration.CurrentLimits.SupplyCurrentLowerTime = 1;
        algaeConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
        algaeConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        algaeConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        algaeConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        algaeConfiguration.Feedback.SensorToMechanismRatio = constants.algaeGearing();
        algaeRollerMotor.getConfigurator().apply(algaeConfiguration);

        final CANrangeConfiguration CANRangeConfiguration = new CANrangeConfiguration();
        CANRangeConfiguration.ToFParams.UpdateMode = UpdateModeValue.ShortRange100Hz;
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
        inputs.coralCANRangeIsDetected = coralCANRangeIsDetected.getValue();
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
}
