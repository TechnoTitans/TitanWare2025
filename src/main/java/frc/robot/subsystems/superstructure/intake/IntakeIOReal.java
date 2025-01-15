package frc.robot.subsystems.superstructure.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.units.measure.*;
import frc.robot.constants.HardwareConstants;

public class IntakeIOReal implements IntakeIO {
    private final HardwareConstants.IntakeConstants constants;

    private final TalonFX pivotMotor;
    private final TalonFX coralRollerMotor;
    private final TalonFX algaeRollerMotor;
    private final CANcoder pivotEncoder;
    private final CANrange coralCANRange;
    private final CANrange algaeCANRange;

    private final MotionMagicExpoTorqueCurrentFOC motionMagicExpoTorqueCurrentFOC;
    private final VelocityTorqueCurrentFOC velocityTorqueCurrentFOC;
    private final TorqueCurrentFOC torqueCurrentFOC;
    private final VoltageOut voltageOut;

    private final StatusSignal<Angle> pivotPosition;
    private final StatusSignal<AngularVelocity> pivotVelocity;
    private final StatusSignal<Voltage> pivotVoltage;
    private final StatusSignal<Current> pivotTorqueCurrent;
    private final StatusSignal<Temperature> pivotDeviceTemp;
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
    private final StatusSignal<Angle> encoderPosition;
    private final StatusSignal<AngularVelocity> encoderVelocity;
    private final StatusSignal<Distance> coralCANRangeDistance;
    private final StatusSignal<Boolean> coralCANRangeIsDetected;
    private final StatusSignal<Distance> algaeCANRangeDistance;
    private final StatusSignal<Boolean> algaeCANRangeIsDetected;

    public IntakeIOReal(final HardwareConstants.IntakeConstants constants) {
        this.constants = constants;

        this.pivotMotor = new TalonFX(constants.intakePivotMotorID(), constants.CANBus());
        this.coralRollerMotor = new TalonFX(constants.coralRollerMotorID(), constants.CANBus());
        this.algaeRollerMotor = new TalonFX(constants.algaeRollerMotorID(), constants.CANBus());
        this.pivotEncoder = new CANcoder(constants.intakePivotCANCoderId(), constants.CANBus());
        this.coralCANRange = new CANrange(constants.coralCANRangeId(), constants.CANBus());
        this.algaeCANRange = new CANrange(constants.algaeCANRangeId(), constants.CANBus());

        this.motionMagicExpoTorqueCurrentFOC = new MotionMagicExpoTorqueCurrentFOC(0);
        this.velocityTorqueCurrentFOC = new VelocityTorqueCurrentFOC(0);
        this.torqueCurrentFOC = new TorqueCurrentFOC(0);
        this.voltageOut = new VoltageOut(0);

        this.pivotPosition = pivotMotor.getPosition();
        this.pivotVelocity = pivotMotor.getVelocity();
        this.pivotVoltage = pivotMotor.getMotorVoltage();
        this.pivotTorqueCurrent = pivotMotor.getTorqueCurrent();
        this.pivotDeviceTemp = pivotMotor.getDeviceTemp();
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
        this.encoderPosition = pivotEncoder.getPosition();
        this.encoderVelocity = pivotEncoder.getVelocity();
        this.coralCANRangeDistance = coralCANRange.getDistance();
        this.coralCANRangeIsDetected = coralCANRange.getIsDetected();
        this.algaeCANRangeDistance = algaeCANRange.getDistance();
        this.algaeCANRangeIsDetected = algaeCANRange.getIsDetected();
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
        algaeCANRange.getConfigurator().apply(CANRangeConfiguration);

        BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                pivotPosition,
                pivotVelocity,
                pivotVoltage,
                pivotTorqueCurrent,
                coralPosition,
                coralVelocity,
                coralVoltage,
                coralTorqueCurrent,
                algaePosition,
                algaeVelocity,
                algaeVoltage,
                algaeTorqueCurrent,
                encoderPosition,
                encoderVelocity,
                coralCANRangeDistance,
                coralCANRangeIsDetected,
                algaeCANRangeDistance,
                algaeCANRangeIsDetected
        );

        BaseStatusSignal.setUpdateFrequencyForAll(
                4,
                pivotDeviceTemp,
                coralDeviceTemp,
                algaeDeviceTemp
        );

        ParentDevice.optimizeBusUtilizationForAll(
                pivotMotor,
                coralRollerMotor,
                algaeRollerMotor,
                pivotEncoder,
                coralCANRange,
                algaeCANRange
        );
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                pivotPosition,
                pivotVelocity,
                pivotVoltage,
                pivotTorqueCurrent,
                pivotDeviceTemp,
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
                encoderPosition,
                encoderVelocity,
                coralCANRangeDistance,
                coralCANRangeIsDetected,
                algaeCANRangeDistance,
                algaeCANRangeIsDetected
        );

        inputs.pivotPositionRots = pivotPosition.getValueAsDouble();
        inputs.pivotVelocityRotsPerSec = pivotVelocity.getValueAsDouble();
        inputs.pivotVoltage = pivotVoltage.getValueAsDouble();
        inputs.pivotTorqueCurrentAmps = pivotTorqueCurrent.getValueAsDouble();
        inputs.pivotTempCelsius = pivotDeviceTemp.getValueAsDouble();
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
        inputs.encoderPositionRots = encoderPosition.getValueAsDouble();
        inputs.encoderVelocityRotsPerSec = encoderVelocity.getValueAsDouble();
        inputs.coralCANRangeDistanceMeters = coralCANRangeDistance.getValueAsDouble();
        inputs.coralCANRangeIsDetected = coralCANRangeIsDetected.getValue();
        inputs.algaeCANRangeDistanceMeters = algaeCANRangeDistance.getValueAsDouble();
        inputs.algaeCANRangeIsDetected = algaeCANRangeIsDetected.getValue();
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
