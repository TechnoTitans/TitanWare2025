package frc.robot.subsystems.superstructure.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.*;
import frc.robot.constants.HardwareConstants;
import frc.robot.utils.ctre.Phoenix6Utils;
import frc.robot.utils.ctre.RefreshAll;

public class ElevatorIOReal implements ElevatorIO {
    private final HardwareConstants.ElevatorConstants constants;

    private final TalonFX masterMotor;
    private final TalonFX followerMotor;

    private final MotionMagicExpoVoltage motionMagicExpoVoltage;
    private final DynamicMotionMagicVoltage dynamicMotionMagicVoltage;
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

    public ElevatorIOReal(final HardwareConstants.ElevatorConstants constants) {
        this.constants = constants;

        this.masterMotor = new TalonFX(constants.rightMotorId(), constants.CANBus());
        this.followerMotor = new TalonFX(constants.leftMotorId(), constants.CANBus());

        this.motionMagicExpoVoltage = new MotionMagicExpoVoltage(0);
        this.dynamicMotionMagicVoltage = new DynamicMotionMagicVoltage(0, 0, 0, 0);
        this.torqueCurrentFOC = new TorqueCurrentFOC(0);
        this.voltageOut = new VoltageOut(0);
        this.follower = new Follower(masterMotor.getDeviceID(), true);

        this.masterPosition = masterMotor.getPosition(false);
        this.masterVelocity = masterMotor.getVelocity(false);
        this.masterVoltage = masterMotor.getMotorVoltage(false);
        this.masterTorqueCurrent = masterMotor.getTorqueCurrent(false);
        this.masterDeviceTemp = masterMotor.getDeviceTemp(false);
        this.followerPosition = followerMotor.getPosition(false);
        this.followerVelocity = followerMotor.getVelocity(false);
        this.followerVoltage = followerMotor.getMotorVoltage(false);
        this.followerTorqueCurrent = followerMotor.getTorqueCurrent(false);
        this.followerDeviceTemp = followerMotor.getDeviceTemp(false);

        RefreshAll.add(
                RefreshAll.CANBus.CANIVORE,
                masterPosition,
                masterVelocity,
                masterVoltage,
                masterTorqueCurrent,
                masterDeviceTemp,
                followerPosition,
                followerVelocity,
                followerVoltage,
                followerTorqueCurrent,
                followerDeviceTemp
        );
    }

    @Override
    public void config() {
        final TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();
        motorConfiguration.Slot0 = new Slot0Configs()
                .withKS(0.037707)
                .withKG(0.52705)
                .withGravityType(GravityTypeValue.Elevator_Static)
                .withKV(0.54587)
                .withKA(0.012141)
                .withKP(15)
                .withKD(1);
        motorConfiguration.MotionMagic.MotionMagicCruiseVelocity = 0;
        motorConfiguration.MotionMagic.MotionMagicExpo_kV = 0.54587;
        motorConfiguration.MotionMagic.MotionMagicExpo_kA = 0.1;
        motorConfiguration.TorqueCurrent.PeakForwardTorqueCurrent = 70;
        motorConfiguration.TorqueCurrent.PeakReverseTorqueCurrent = -70;
        motorConfiguration.CurrentLimits.StatorCurrentLimit = 70;
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
                followerTorqueCurrent
        );
        BaseStatusSignal.setUpdateFrequencyForAll(
                4,
                masterDeviceTemp,
                followerDeviceTemp
        );
        ParentDevice.optimizeBusUtilizationForAll(
                4,
                masterMotor,
                followerMotor
        );
    }

    @Override
    public void updateInputs(final ElevatorIOInputs inputs) {
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
    }

    @Override
    public void setPosition(final double positionRots) {
        Phoenix6Utils.reportIfNotOk(masterMotor, masterMotor.setPosition(positionRots));
        Phoenix6Utils.reportIfNotOk(followerMotor, followerMotor.setPosition(positionRots));
    }

    @Override
    public void toPosition(final double positionRots) {
        masterMotor.setControl(motionMagicExpoVoltage.withPosition(positionRots));
        followerMotor.setControl(follower);
    }

    @Override
    public void toPosition(
            final double positionRots,
            final double velocityRotsPerSec,
            final double accelerationRotsPerSec2
    ) {
        masterMotor.setControl(dynamicMotionMagicVoltage
                .withPosition(positionRots)
                .withVelocity(velocityRotsPerSec)
                .withAcceleration(accelerationRotsPerSec2)
        );
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
