package frc.robot.subsystems.superstructure.arm.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.units.measure.*;
import frc.robot.constants.HardwareConstants;

public class IntakeArmIOReal implements IntakeArmIO {
    private final HardwareConstants.IntakeArmConstants constants;

    private final TalonFXS pivotMotor;
    private final CANcoder pivotEncoder;

    private final MotionMagicExpoVoltage motionMagicExpoVoltage;
    private final VoltageOut voltageOut;

    private final MotionMagicConfigs motionMagicConfigs;

    private final StatusSignal<Angle> pivotPosition;
    private final StatusSignal<AngularVelocity> pivotVelocity;
    private final StatusSignal<Voltage> pivotVoltage;
    private final StatusSignal<Current> pivotTorqueCurrent;
    private final StatusSignal<Temperature> pivotDeviceTemp;
    private final StatusSignal<Angle> encoderPosition;
    private final StatusSignal<AngularVelocity> encoderVelocity;

    public IntakeArmIOReal(final HardwareConstants.IntakeArmConstants constants) {
        this.constants = constants;

        this.pivotMotor = new TalonFXS(constants.intakePivotMotorID(), constants.CANBus());
        this.pivotEncoder = new CANcoder(constants.intakePivotCANCoderId(), constants.CANBus());

        this.motionMagicExpoVoltage = new MotionMagicExpoVoltage(0);
        this.voltageOut = new VoltageOut(0);

        this.motionMagicConfigs = new MotionMagicConfigs();

        this.pivotPosition = pivotMotor.getPosition();
        this.pivotVelocity = pivotMotor.getVelocity();
        this.pivotVoltage = pivotMotor.getMotorVoltage();
        this.pivotTorqueCurrent = pivotMotor.getTorqueCurrent();
        this.pivotDeviceTemp = pivotMotor.getDeviceTemp();
        this.encoderPosition = pivotEncoder.getPosition();
        this.encoderVelocity = pivotEncoder.getVelocity();

        this.pivotMotor.setPosition(0);
    }

    @Override
    public void config() {
        final CANcoderConfiguration encoderConfiguration = new CANcoderConfiguration();
        encoderConfiguration.MagnetSensor.MagnetOffset = constants.intakePivotCANCoderOffset();
        encoderConfiguration.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        pivotEncoder.getConfigurator().apply(encoderConfiguration);

        final TalonFXSConfiguration pivotConfiguration = new TalonFXSConfiguration();
        pivotConfiguration.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
        pivotConfiguration.Commutation.AdvancedHallSupport = AdvancedHallSupportValue.Enabled;
        pivotConfiguration.Slot0 = new Slot0Configs()
                .withKS(0.3415)
                .withKG(0.1083)
                .withGravityType(GravityTypeValue.Elevator_Static)
                .withKV(5.1579)
                .withKA(0.1686)
                .withKP(57.24)
                .withKD(1);
        motionMagicConfigs.MotionMagicCruiseVelocity = 0;
        motionMagicConfigs.MotionMagicExpo_kV = 5;
        motionMagicConfigs.MotionMagicExpo_kA = 0.5;
        pivotConfiguration.MotionMagic = motionMagicConfigs;
        pivotConfiguration.CurrentLimits.StatorCurrentLimit = 60;
        pivotConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        pivotConfiguration.CurrentLimits.SupplyCurrentLimit = 50;
        pivotConfiguration.CurrentLimits.SupplyCurrentLowerLimit = 40;
        pivotConfiguration.CurrentLimits.SupplyCurrentLowerTime = 1;
        pivotConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
        pivotConfiguration.ExternalFeedback.ExternalFeedbackSensorSource = ExternalFeedbackSensorSourceValue.FusedCANcoder;
        pivotConfiguration.ExternalFeedback.FeedbackRemoteSensorID = pivotEncoder.getDeviceID();
        pivotConfiguration.ExternalFeedback.RotorToSensorRatio = constants.pivotGearing();
        pivotConfiguration.ExternalFeedback.SensorToMechanismRatio = 1;
        pivotConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        pivotConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        pivotConfiguration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = constants.pivotUpperLimitRots();
        pivotConfiguration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        pivotConfiguration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = constants.pivotLowerLimitRots();
        pivotConfiguration.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        pivotMotor.getConfigurator().apply(pivotConfiguration);

        BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                pivotPosition,
                pivotVelocity,
                pivotVoltage,
                pivotTorqueCurrent,
                encoderPosition,
                encoderVelocity
        );

        BaseStatusSignal.setUpdateFrequencyForAll(
                4,
                pivotDeviceTemp
        );

        ParentDevice.optimizeBusUtilizationForAll(
                pivotMotor,
                pivotEncoder
        );
    }

    @Override
    public void updateInputs(IntakeArmIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                pivotPosition,
                pivotVelocity,
                pivotVoltage,
                pivotTorqueCurrent,
                pivotDeviceTemp,
                encoderPosition,
                encoderVelocity
        );

        inputs.pivotPositionRots = pivotPosition.getValueAsDouble();
        inputs.pivotVelocityRotsPerSec = pivotVelocity.getValueAsDouble();
        inputs.pivotVoltage = pivotVoltage.getValueAsDouble();
        inputs.pivotTorqueCurrentAmps = pivotTorqueCurrent.getValueAsDouble();
        inputs.pivotTempCelsius = pivotDeviceTemp.getValueAsDouble();
        inputs.encoderPositionRots = encoderPosition.getValueAsDouble();
        inputs.encoderVelocityRotsPerSec = encoderVelocity.getValueAsDouble();
    }

    @Override
    public void setMotionMagicCruiseVelocity(final double cruiseVelocity) {
        if (motionMagicConfigs.MotionMagicCruiseVelocity != cruiseVelocity) {
            motionMagicConfigs.MotionMagicCruiseVelocity = cruiseVelocity;
            pivotMotor.getConfigurator().apply(motionMagicConfigs);
        }
    }

    @Override
    public void toPivotPosition(final double pivotPositionRots) {
        pivotMotor.setControl(motionMagicExpoVoltage.withPosition(pivotPositionRots));
    }

    @Override
    public void toPivotVoltage(final double volts) {
        pivotMotor.setControl(voltageOut.withOutput(volts));
    }
}
