package frc.robot.subsystems.superstructure.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.units.measure.*;
import frc.robot.constants.HardwareConstants;
import frc.robot.utils.ctre.Phoenix6Utils;

public class ElevatorIOReal implements ElevatorIO {
    private final HardwareConstants.ElevatorConstants constants;

    private final TalonFX masterMotor;
    private final TalonFX followerMotor;
    private final CANrange canRange;

    private final MotionMagicExpoTorqueCurrentFOC motionMagicExpoTorqueCurrentFOC;
    private final TorqueCurrentFOC torqueCurrentFOC;
    private final VoltageOut voltageOut;
    private final Follower follower;

    private final StatusSignal<Angle> masterPosition;
    private final StatusSignal<AngularVelocity> masterVelocity;
    private final StatusSignal<Voltage> masterVoltage;
    private final StatusSignal<Current> masterTorqueCurrent;
    private final StatusSignal<Temperature> masterDeviceTemp;
    private final StatusSignal<Angle> followerPosition;
    private final StatusSignal<AngularVelocity> followerVelocity;
    private final StatusSignal<Voltage> followerVoltage;
    private final StatusSignal<Current> followerTorqueCurrent;
    private final StatusSignal<Temperature> followerDeviceTemp;
    private final StatusSignal<Distance> canRangeDistance;
    private final StatusSignal<Boolean> canRangeIsDetected;

    public ElevatorIOReal(final HardwareConstants.ElevatorConstants constants) {
        this.constants = constants;

        this.masterMotor = new TalonFX(constants.rightMotorId(), constants.CANBus());
        this.followerMotor = new TalonFX(constants.leftMotorId(), constants.CANBus());
        this.canRange = new CANrange(constants.CANrangeId(), constants.CANBus());

        this.motionMagicExpoTorqueCurrentFOC = new MotionMagicExpoTorqueCurrentFOC(0);
        this.torqueCurrentFOC = new TorqueCurrentFOC(0);
        this.voltageOut = new VoltageOut(0);
        this.follower = new Follower(masterMotor.getDeviceID(), false);

        this.masterPosition = masterMotor.getPosition();
        this.masterVelocity = masterMotor.getVelocity();
        this.masterVoltage = masterMotor.getMotorVoltage();
        this.masterTorqueCurrent = masterMotor.getTorqueCurrent();
        this.masterDeviceTemp = masterMotor.getDeviceTemp();
        this.followerPosition = followerMotor.getPosition();
        this.followerVelocity = followerMotor.getVelocity();
        this.followerVoltage = followerMotor.getMotorVoltage();
        this.followerTorqueCurrent = followerMotor.getTorqueCurrent();
        this.followerDeviceTemp = followerMotor.getDeviceTemp();
        this.canRangeDistance = canRange.getDistance();
        this.canRangeIsDetected = canRange.getIsDetected();
    }

    @Override
    public void config() {
        final CANrangeConfiguration CANRangeConfiguration = new CANrangeConfiguration();
        CANRangeConfiguration.ToFParams.UpdateMode = UpdateModeValue.LongRangeUserFreq;
        CANRangeConfiguration.ProximityParams.ProximityThreshold = 0.01;
        CANRangeConfiguration.ProximityParams.ProximityHysteresis = 0.03;
        canRange.getConfigurator().apply(CANRangeConfiguration);

        final TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();
        motorConfiguration.Slot0 = new Slot0Configs()
                .withKS(0)
                .withKG(0.11)
                .withGravityType(GravityTypeValue.Elevator_Static)
                .withKV(13.97)
                .withKA(0.015)
                .withKP(50);
        motorConfiguration.MotionMagic.MotionMagicCruiseVelocity = 0;
        motorConfiguration.MotionMagic.MotionMagicExpo_kV = 13.97;
        motorConfiguration.MotionMagic.MotionMagicExpo_kA = 0.015;
        motorConfiguration.TorqueCurrent.PeakForwardTorqueCurrent = 80;
        motorConfiguration.TorqueCurrent.PeakReverseTorqueCurrent = -80;
        motorConfiguration.CurrentLimits.StatorCurrentLimit = 60;
        motorConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        motorConfiguration.Feedback.SensorToMechanismRatio = constants.gearing();
        motorConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        motorConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        motorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motorConfiguration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = constants.upperLimitRots();
        motorConfiguration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        motorConfiguration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = constants.lowerLimitRots();
        motorConfiguration.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        masterMotor.getConfigurator().apply(motorConfiguration);

        motorConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        followerMotor.getConfigurator().apply(motorConfiguration);

        BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                masterPosition,
                masterVelocity,
                masterVoltage,
                masterTorqueCurrent,
                followerPosition,
                followerVelocity,
                followerVoltage,
                followerTorqueCurrent,
                canRangeDistance,
                canRangeIsDetected
        );
        BaseStatusSignal.setUpdateFrequencyForAll(
                4,
                masterDeviceTemp,
                followerDeviceTemp
        );
        ParentDevice.optimizeBusUtilizationForAll(
                masterMotor,
                followerMotor,
                canRange
        );
    }

    @Override
    public void updateInputs(final ElevatorIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                masterPosition,
                masterVelocity,
                masterVoltage,
                masterTorqueCurrent,
                masterDeviceTemp,
                followerPosition,
                followerVelocity,
                followerVoltage,
                followerTorqueCurrent,
                followerDeviceTemp,
                canRangeDistance,
                canRangeIsDetected
        );

        inputs.masterPositionRots = masterPosition.getValueAsDouble();
        inputs.masterVelocityRotsPerSec = masterVelocity.getValueAsDouble();
        inputs.masterVoltage = masterVoltage.getValueAsDouble();
        inputs.masterTorqueCurrentAmps = masterTorqueCurrent.getValueAsDouble();
        inputs.masterTempCelsius = masterDeviceTemp.getValueAsDouble();
        inputs.followerPositionRots = followerPosition.getValueAsDouble();
        inputs.followerVelocityRotsPerSec = followerVelocity.getValueAsDouble();
        inputs.followerVoltage = followerVoltage.getValueAsDouble();
        inputs.followerTorqueCurrentAmps = followerTorqueCurrent.getValueAsDouble();
        inputs.followerTempCelsius = followerDeviceTemp.getValueAsDouble();
        inputs.canRangeDistanceMeters = canRangeDistance.getValueAsDouble();
        inputs.canRangeIsDetected = canRangeIsDetected.getValue();
    }

    @Override
    public void setPosition(final double positionRots) {
        Phoenix6Utils.reportIfNotOk(masterMotor, masterMotor.setPosition(positionRots));
        Phoenix6Utils.reportIfNotOk(followerMotor, followerMotor.setPosition(positionRots));
    }

    @Override
    public void toPosition(final double positionRots) {
        masterMotor.setControl(motionMagicExpoTorqueCurrentFOC.withPosition(positionRots));
        followerMotor.setControl(follower);
    }

    @Override
    public void toVoltage(final double volts) {
        masterMotor.setControl(voltageOut.withOutput(volts));
        followerMotor.setControl(follower);
    }

    @Override
    public void toTorqueCurrent(final double torqueCurrentAmps) {
        masterMotor.setControl(torqueCurrentFOC.withOutput(torqueCurrentAmps));
        followerMotor.setControl(follower);
    }
}
