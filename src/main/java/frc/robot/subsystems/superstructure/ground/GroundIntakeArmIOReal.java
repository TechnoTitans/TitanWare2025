package frc.robot.subsystems.superstructure.ground;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
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

    private final StatusSignal<Angle> pivotPosition;
    private final StatusSignal<AngularVelocity> pivotVelocity;
    private final StatusSignal<Voltage> pivotVoltage;
    private final StatusSignal<Current> pivotTorqueCurrent;
    private final StatusSignal<Temperature> pivotDeviceTemp;
    private final StatusSignal<Angle> encoderPosition;
    private final StatusSignal<AngularVelocity> encoderVelocity;

    private final MotionMagicExpoVoltage motionMagicExpoVoltage;
    private final VoltageOut voltageOut;
    private final TorqueCurrentFOC torqueCurrentFOC;

    public GroundIntakeArmIOReal(final HardwareConstants.GroundIntakeArmConstants constants) {
        this.constants = constants;

        this.pivotMotor = new TalonFX(constants.pivotMotorID(), constants.CANBus());
        this.pivotEncoder = new CANcoder(constants.groundIntakePivotEncoderID(), constants.CANBus());

        this.pivotPosition = pivotMotor.getPosition(false);
        this.pivotVelocity = pivotMotor.getVelocity(false);
        this.pivotVoltage = pivotMotor.getMotorVoltage(false);
        this.pivotTorqueCurrent = pivotMotor.getTorqueCurrent(false);
        this.pivotDeviceTemp = pivotMotor.getDeviceTemp(false);
        this.encoderPosition = pivotEncoder.getPosition(false);
        this.encoderVelocity = pivotEncoder.getVelocity(false);

        this.motionMagicExpoVoltage = new MotionMagicExpoVoltage(0);
        this.voltageOut = new VoltageOut(0);
        this.torqueCurrentFOC = new TorqueCurrentFOC(0);

        RefreshAll.add(
                RefreshAll.CANBus.RIO,
                pivotPosition,
                pivotVelocity,
                pivotVoltage,
                pivotTorqueCurrent,
                pivotDeviceTemp,
                encoderPosition,
                encoderVelocity
        );
    }

    @Override
    public void config() {
        final CANcoderConfiguration pivotCANCoderConfig = new CANcoderConfiguration();
        pivotCANCoderConfig.MagnetSensor.MagnetOffset = constants.groundIntakePivotEncoderOffset();
        pivotCANCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        pivotEncoder.getConfigurator().apply(pivotCANCoderConfig);

        final TalonFXConfiguration pivotMotorConfig = new TalonFXConfiguration();
        pivotMotorConfig.Slot0 = new Slot0Configs()
                .withKS(0.27721)
                .withKG(0.4366)
                .withGravityType(GravityTypeValue.Arm_Cosine)
                .withKV(39.374)
                .withKA(0.55684)
                .withKP(61.772)
                .withKD(0);
        pivotMotorConfig.MotionMagic.MotionMagicCruiseVelocity = 0;
        pivotMotorConfig.MotionMagic.MotionMagicExpo_kV = 39.374;
        pivotMotorConfig.MotionMagic.MotionMagicExpo_kA = 10;
        pivotMotorConfig.TorqueCurrent.PeakForwardTorqueCurrent = 80;
        pivotMotorConfig.TorqueCurrent.PeakReverseTorqueCurrent = -80;
        pivotMotorConfig.CurrentLimits.StatorCurrentLimit = 80;
        pivotMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        pivotMotorConfig.CurrentLimits.SupplyCurrentLimit = 50;
        pivotMotorConfig.CurrentLimits.SupplyCurrentLowerLimit = 40;
        pivotMotorConfig.CurrentLimits.SupplyCurrentLowerTime = 1;
        pivotMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        pivotMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        pivotMotorConfig.Feedback.FeedbackRemoteSensorID = pivotEncoder.getDeviceID();
        pivotMotorConfig.Feedback.SensorToMechanismRatio = 1;
        pivotMotorConfig.Feedback.RotorToSensorRatio = constants.pivotGearing();
        pivotMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
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
                encoderPosition,
                encoderVelocity
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
    public void updateInputs(GroundIntakeArmIOInputs inputs) {
        inputs.pivotPositionRots = pivotPosition.getValueAsDouble();
        inputs.pivotVelocityRotsPerSec = pivotVelocity.getValueAsDouble();
        inputs.pivotVoltage = pivotVoltage.getValueAsDouble();
        inputs.pivotTorqueCurrentAmps = pivotTorqueCurrent.getValueAsDouble();
        inputs.pivotTempCelsius = pivotDeviceTemp.getValueAsDouble();
        inputs.encoderPositionRots = encoderPosition.getValueAsDouble();
        inputs.encoderVelocityRotsPerSec = encoderVelocity.getValueAsDouble();
    }

    @Override
    public void toPivotPosition(double positionRots) {
        pivotMotor.setControl(motionMagicExpoVoltage.withPosition(positionRots));
    }

    @Override
    public void toPivotVoltage(double volts) {
        pivotMotor.setControl(voltageOut.withOutput(volts));
    }

    @Override
    public void toPivotTorqueCurrent(double torqueCurrent) {
        pivotMotor.setControl(torqueCurrentFOC.withOutput(torqueCurrent));
    }
}
