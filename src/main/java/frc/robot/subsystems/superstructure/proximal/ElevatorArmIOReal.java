package frc.robot.subsystems.superstructure.proximal;

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

public class ElevatorArmIOReal implements ElevatorArmIO {
    private final HardwareConstants.ElevatorArmConstants constants;

    private final TalonFX pivotMotor;
    private final CANcoder pivotCANCoder;

    private final MotionMagicExpoVoltage motionMagicExpoVoltage;
    private final TorqueCurrentFOC torqueCurrentFOC;
    private final VoltageOut voltageOut;

    private final StatusSignal<Angle> pivotPosition;
    private final StatusSignal<AngularVelocity> pivotVelocity;
    private final StatusSignal<Voltage> pivotVoltage;
    private final StatusSignal<Current> pivotTorqueCurrent;
    private final StatusSignal<Temperature> pivotDeviceTemp;
    private final StatusSignal<Angle> pivotCANCoderPosition;
    private final StatusSignal<AngularVelocity> pivotCANCoderVelocity;

    public ElevatorArmIOReal(final HardwareConstants.ElevatorArmConstants constants) {
        this.constants = constants;

        this.pivotMotor = new TalonFX(constants.motorId(), constants.CANBus());
        this.pivotCANCoder = new CANcoder(constants.CANCoderId(), constants.CANBus());

        this.motionMagicExpoVoltage = new MotionMagicExpoVoltage(0);
        this.torqueCurrentFOC = new TorqueCurrentFOC(0);
        this.voltageOut = new VoltageOut(0);

        this.pivotPosition = pivotMotor.getPosition();
        this.pivotVelocity = pivotMotor.getVelocity();
        this.pivotVoltage = pivotMotor.getMotorVoltage();
        this.pivotTorqueCurrent = pivotMotor.getTorqueCurrent();
        this.pivotDeviceTemp = pivotMotor.getDeviceTemp();
        this.pivotCANCoderPosition = pivotCANCoder.getPosition();
        this.pivotCANCoderVelocity = pivotCANCoder.getVelocity();

        RefreshAll.add(
                RefreshAll.CANBus.RIO,
                pivotPosition,
                pivotVelocity,
                pivotVoltage,
                pivotTorqueCurrent,
                pivotCANCoderPosition,
                pivotCANCoderVelocity
        );
    }

    @Override
    public void config() {
        final CANcoderConfiguration pivotCANCoderConfig = new CANcoderConfiguration();
        pivotCANCoderConfig.MagnetSensor.MagnetOffset = constants.CANCoderOffset();
        pivotCANCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        pivotCANCoder.getConfigurator().apply(pivotCANCoderConfig);

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
        pivotMotorConfig.Feedback.FeedbackRemoteSensorID = pivotCANCoder.getDeviceID();
        pivotMotorConfig.Feedback.SensorToMechanismRatio = 1;
        pivotMotorConfig.Feedback.RotorToSensorRatio = constants.gearing();
        pivotMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        pivotMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        pivotMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = constants.upperLimitRots();
        pivotMotorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        pivotMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = constants.lowerLimitRots();
        pivotMotorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        pivotMotor.getConfigurator().apply(pivotMotorConfig);

        BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                pivotPosition,
                pivotVelocity,
                pivotVoltage,
                pivotTorqueCurrent,
                pivotCANCoderPosition,
                pivotCANCoderVelocity
        );
        BaseStatusSignal.setUpdateFrequencyForAll(
                4,
                pivotDeviceTemp
        );
        ParentDevice.optimizeBusUtilizationForAll(
                4,
                pivotMotor,
                pivotCANCoder
        );
    }

    @Override
    public void updateInputs(final ElevatorArmIOInputs inputs) {
        inputs.pivotPositionRots = pivotPosition.getValueAsDouble();
        inputs.pivotVelocityRotsPerSec = pivotVelocity.getValueAsDouble();
        inputs.pivotVoltageVolts = pivotVoltage.getValueAsDouble();
        inputs.pivotTorqueCurrentAmps = pivotTorqueCurrent.getValueAsDouble();
        inputs.pivotTempCelsius = pivotDeviceTemp.getValueAsDouble();
        inputs.pivotEncoderPositionRots = pivotCANCoderPosition.getValueAsDouble();
        inputs.pivotEncoderVelocityRotsPerSec = pivotCANCoderVelocity.getValueAsDouble();
    }

    @Override
    public void toPivotPosition(final double pivotPositionRots) {
        pivotMotor.setControl(motionMagicExpoVoltage.withPosition(pivotPositionRots));
    }

    @Override
    public void toPivotVoltage(final double pivotVolts) {
        pivotMotor.setControl(voltageOut.withOutput(pivotVolts));
    }

    @Override
    public void toPivotTorqueCurrent(final double pivotTorqueCurrentAmps) {
        pivotMotor.setControl(torqueCurrentFOC.withOutput(pivotTorqueCurrentAmps));
    }
}
