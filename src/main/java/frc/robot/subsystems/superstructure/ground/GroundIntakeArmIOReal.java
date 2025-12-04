package frc.robot.subsystems.superstructure.ground;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.units.measure.*;
import frc.robot.constants.HardwareConstants;
import frc.robot.utils.ctre.RefreshAll;

public class GroundIntakeArmIOReal implements GroundIntakeArmIO {
    private final HardwareConstants.GroundIntakeArmConstants constants;

    private final TalonFX pivotMotor;
    private final CANcoder pivotEncoder;

    private final MotionMagicExpoVoltage motionMagicExpoVoltage;
    private final PositionVoltage positionVoltage;
    private final VoltageOut voltageOut;

    private final StatusSignal<Angle> pivotPosition;
    private final StatusSignal<AngularVelocity> pivotVelocity;
    private final StatusSignal<Voltage> pivotVoltage;
    private final StatusSignal<Current> pivotTorqueCurrent;
    private final StatusSignal<Temperature> pivotDeviceTemp;
    private final StatusSignal<Angle> pivotEncoderPosition;
    private final StatusSignal<AngularVelocity> pivotEncoderVelocity;

    public GroundIntakeArmIOReal(final HardwareConstants.GroundIntakeArmConstants constants) {
        this.constants = constants;

        final HardwareConstants.CANBus bus = constants.CANBus();
        final CANBus p6Bus = bus.toPhoenix6CANBus();
        this.pivotMotor = new TalonFX(constants.pivotMotorID(), p6Bus);
        this.pivotEncoder = new CANcoder(constants.pivotCANCoderId(), p6Bus);

        this.motionMagicExpoVoltage = new MotionMagicExpoVoltage(0);
        this.positionVoltage = new PositionVoltage(0);
        this.voltageOut = new VoltageOut(0);

        this.pivotPosition = pivotMotor.getPosition(false);
        this.pivotVelocity = pivotMotor.getVelocity(false);
        this.pivotVoltage = pivotMotor.getMotorVoltage(false);
        this.pivotTorqueCurrent = pivotMotor.getTorqueCurrent(false);
        this.pivotDeviceTemp = pivotMotor.getDeviceTemp(false);
        this.pivotEncoderPosition = pivotEncoder.getPosition(false);
        this.pivotEncoderVelocity = pivotEncoder.getVelocity(false);

        RefreshAll.add(
                bus,
                pivotPosition,
                pivotVelocity,
                pivotVoltage,
                pivotTorqueCurrent,
                pivotDeviceTemp,
                pivotEncoderPosition,
                pivotEncoderVelocity
        );
    }

    @Override
    public void config() {
        final CANcoderConfiguration encoderConfiguration = new CANcoderConfiguration();
        encoderConfiguration.MagnetSensor.MagnetOffset = constants.pivotCANCoderOffset();
        encoderConfiguration.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        pivotEncoder.getConfigurator().apply(encoderConfiguration);

        // TODO gains + config
        final TalonFXConfiguration pivotMotorConfig = new TalonFXConfiguration();
        pivotMotorConfig.Slot0 = new Slot0Configs()
                .withKS(0)
                .withKG(0.25)
                .withGravityType(GravityTypeValue.Arm_Cosine)
                .withKV(9.56)
                .withKA(0.08)
                .withKP(86.19)
                .withKD(1.47);
        pivotMotorConfig.MotionMagic.MotionMagicCruiseVelocity = 0;
        pivotMotorConfig.MotionMagic.MotionMagicExpo_kV = 40;
        pivotMotorConfig.MotionMagic.MotionMagicExpo_kA = 10;
        pivotMotorConfig.TorqueCurrent.PeakForwardTorqueCurrent = 60;
        pivotMotorConfig.TorqueCurrent.PeakReverseTorqueCurrent = -60;
        pivotMotorConfig.CurrentLimits.StatorCurrentLimit = 60;
        pivotMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        pivotMotorConfig.CurrentLimits.SupplyCurrentLimit = 50;
        pivotMotorConfig.CurrentLimits.SupplyCurrentLowerLimit = 40;
        pivotMotorConfig.CurrentLimits.SupplyCurrentLowerTime = 1;
        pivotMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        pivotMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        pivotMotorConfig.Feedback.FeedbackRemoteSensorID = pivotEncoder.getDeviceID();
        pivotMotorConfig.Feedback.SensorToMechanismRatio = 1;
        pivotMotorConfig.Feedback.RotorToSensorRatio = constants.pivotGearing();
        pivotMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        pivotMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        pivotMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = constants.pivotUpperLimitRots();
        pivotMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        pivotMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = constants.pivotLowerLimitRots();
        pivotMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        pivotMotor.getConfigurator().apply(pivotMotorConfig);

        BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                pivotPosition,
                pivotVelocity,
                pivotVoltage,
                pivotTorqueCurrent,
                pivotEncoderPosition,
                pivotEncoderVelocity
        );
        BaseStatusSignal.setUpdateFrequencyForAll(
                4,
                pivotDeviceTemp
        );
        ParentDevice.optimizeBusUtilizationForAll(
                4,
                pivotMotor,
                pivotEncoder
        );
    }

    @Override
    public void updateInputs(final GroundIntakeArmIO.GroundIntakeArmIOInputs inputs) {
        inputs.pivotPositionRots = pivotPosition.getValueAsDouble();
        inputs.pivotVelocityRotsPerSec = pivotVelocity.getValueAsDouble();
        inputs.pivotVoltage = pivotVoltage.getValueAsDouble();
        inputs.pivotTorqueCurrentAmps = pivotTorqueCurrent.getValueAsDouble();
        inputs.pivotTempCelsius = pivotDeviceTemp.getValueAsDouble();
        inputs.encoderPositionRots = pivotEncoderPosition.getValueAsDouble();
        inputs.encoderVelocityRotsPerSec = pivotEncoderVelocity.getValueAsDouble();
    }

    @Override
    public void toPivotPosition(final double pivotPositionRots) {
        pivotMotor.setControl(motionMagicExpoVoltage.withPosition(pivotPositionRots));
    }

    @Override
    public void toPivotPositionUnprofiled(final double pivotPositionRots, final double pivotVelocityRotsPerSec) {
        pivotMotor.setControl(positionVoltage
                .withPosition(pivotPositionRots)
                .withVelocity(pivotVelocityRotsPerSec)
        );
    }

    @Override
    public void toPivotVoltage(final double volts) {
        pivotMotor.setControl(voltageOut.withOutput(volts));
    }
}
