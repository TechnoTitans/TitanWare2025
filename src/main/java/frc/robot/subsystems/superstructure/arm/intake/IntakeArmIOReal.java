package frc.robot.subsystems.superstructure.arm.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.units.measure.*;
import frc.robot.constants.HardwareConstants;

public class IntakeArmIOReal implements IntakeArmIO {
    private final HardwareConstants.IntakeArmConstants constants;

    private final TalonFX pivotMotor;
    private final CANcoder pivotEncoder;

    private final MotionMagicExpoTorqueCurrentFOC motionMagicExpoTorqueCurrentFOC;
    private final TorqueCurrentFOC torqueCurrentFOC;
    private final VoltageOut voltageOut;

    private final StatusSignal<Angle> pivotPosition;
    private final StatusSignal<AngularVelocity> pivotVelocity;
    private final StatusSignal<Voltage> pivotVoltage;
    private final StatusSignal<Current> pivotTorqueCurrent;
    private final StatusSignal<Temperature> pivotDeviceTemp;
    private final StatusSignal<Angle> encoderPosition;
    private final StatusSignal<AngularVelocity> encoderVelocity;

    public IntakeArmIOReal(final HardwareConstants.IntakeArmConstants constants) {
        this.constants = constants;

        this.pivotMotor = new TalonFX(constants.intakePivotMotorID(), constants.CANBus());
        this.pivotEncoder = new CANcoder(constants.intakePivotCANCoderId(), constants.CANBus());

        this.motionMagicExpoTorqueCurrentFOC = new MotionMagicExpoTorqueCurrentFOC(0);
        this.torqueCurrentFOC = new TorqueCurrentFOC(0);
        this.voltageOut = new VoltageOut(0);

        this.pivotPosition = pivotMotor.getPosition();
        this.pivotVelocity = pivotMotor.getVelocity();
        this.pivotVoltage = pivotMotor.getMotorVoltage();
        this.pivotTorqueCurrent = pivotMotor.getTorqueCurrent();
        this.pivotDeviceTemp = pivotMotor.getDeviceTemp();
        this.encoderPosition = pivotEncoder.getPosition();
        this.encoderVelocity = pivotEncoder.getVelocity();
    }

    @Override
    public void config() {
        final CANcoderConfiguration encoderConfiguration = new CANcoderConfiguration();
        encoderConfiguration.MagnetSensor.MagnetOffset = constants.intakePivotCANCoderOffset();
        encoderConfiguration.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        pivotEncoder.getConfigurator().apply(encoderConfiguration);

        final TalonFXConfiguration pivotConfiguration = new TalonFXConfiguration();
        pivotConfiguration.Slot0 = new Slot0Configs()
                .withKS(0)
                .withKG(0.11)
                .withGravityType(GravityTypeValue.Arm_Cosine)
                .withKV(13.97)
                .withKA(0.015)
                .withKP(50);
        pivotConfiguration.MotionMagic.MotionMagicCruiseVelocity = 0;
        pivotConfiguration.MotionMagic.MotionMagicExpo_kV = 13.97;
        pivotConfiguration.MotionMagic.MotionMagicExpo_kA = 0.015;
        pivotConfiguration.TorqueCurrent.PeakForwardTorqueCurrent = 80;
        pivotConfiguration.TorqueCurrent.PeakReverseTorqueCurrent = -80;
        pivotConfiguration.CurrentLimits.StatorCurrentLimit = 60;
        pivotConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        pivotConfiguration.CurrentLimits.SupplyCurrentLimit = 50;
        pivotConfiguration.CurrentLimits.SupplyCurrentLowerLimit = 40;
        pivotConfiguration.CurrentLimits.SupplyCurrentLowerTime = 1;
        pivotConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
        pivotConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        pivotConfiguration.Feedback.FeedbackRemoteSensorID = pivotEncoder.getDeviceID();
        pivotConfiguration.Feedback.RotorToSensorRatio = constants.pivotGearing();
        pivotConfiguration.Feedback.SensorToMechanismRatio = 1;
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
    public void toPivotPosition(final double pivotPositionRots) {
        pivotMotor.setControl(motionMagicExpoTorqueCurrentFOC.withPosition(pivotPositionRots));
    }
    @Override
    public void toPivotVoltage(final double volts) {
        pivotMotor.setControl(voltageOut.withOutput(volts));
    }
    @Override
    public void toPivotTorqueCurrent(final double torqueCurrentAmps) {
        pivotMotor.setControl(torqueCurrentFOC.withOutput(torqueCurrentAmps));
    }
}
