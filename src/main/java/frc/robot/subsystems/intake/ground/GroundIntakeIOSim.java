package frc.robot.subsystems.intake.ground;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
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
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.constants.HardwareConstants;
import frc.robot.utils.MoreDCMotor;
import frc.robot.utils.closeables.ToClose;
import frc.robot.utils.control.DeltaTime;
import frc.robot.utils.ctre.RefreshAll;
import frc.robot.utils.sim.motors.TalonFXSim;

public class GroundIntakeIOSim implements GroundIntakeIO  {
    private static final double SIM_UPDATE_PERIOD_SEC = 0.005;

    private final DeltaTime deltaTime;
    private final HardwareConstants.GroundIntakeConstants constants;

    private final TalonFX motor;
    private final CANrange coralCANRange;

    private final TalonFXSim motorSim;

    private final VelocityTorqueCurrentFOC velocityTorqueCurrentFOC;
    private final TorqueCurrentFOC torqueCurrentFOC;
    private final VoltageOut voltageOut;

    private final StatusSignal<Angle> wheelPosition;
    private final StatusSignal<AngularVelocity> wheelVelocity;
    private final StatusSignal<Voltage> wheelVoltage;
    private final StatusSignal<Current> wheelTorqueCurrent;
    private final StatusSignal<Temperature> wheelDeviceTemp;
    private final StatusSignal<Distance> coralCANRangeDistance;
    private final StatusSignal<Boolean> coralDetected;

    public GroundIntakeIOSim(final HardwareConstants.GroundIntakeConstants constants) {
        this.deltaTime = new DeltaTime(true);
        this.constants = constants;

        final HardwareConstants.CANBus bus = constants.CANBus();
        final CANBus p6Bus = bus.toPhoenix6CANBus();
        this.motor = new TalonFX(constants.wheelMotorID(), p6Bus);
        this.coralCANRange = new CANrange(constants.coralCANRangeID(), p6Bus);

        final DCMotorSim motorSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                        0.19557 / (2 * Math.PI),
                        2.9856 / (2 * Math.PI)
                ),
                MoreDCMotor.getKrakenX44(1)
        );
        this.motorSim = new TalonFXSim(
                motor,
                constants.wheelGearing(),
                motorSim::update,
                motorSim::setInputVoltage,
                motorSim::getAngularPositionRad,
                motorSim::getAngularVelocityRadPerSec
        );

        this.velocityTorqueCurrentFOC = new VelocityTorqueCurrentFOC(0);
        this.torqueCurrentFOC = new TorqueCurrentFOC(0);
        this.voltageOut = new VoltageOut(0);

        this.wheelPosition = motor.getPosition(false);
        this.wheelVelocity = motor.getVelocity(false);
        this.wheelVoltage = motor.getMotorVoltage(false);
        this.wheelTorqueCurrent = motor.getTorqueCurrent(false);
        this.wheelDeviceTemp = motor.getDeviceTemp(false);
        this.coralCANRangeDistance = coralCANRange.getDistance(false);
        this.coralDetected = coralCANRange.getIsDetected(false);

        RefreshAll.add(
                bus,
                wheelPosition,
                wheelVelocity,
                wheelVoltage,
                wheelTorqueCurrent,
                wheelDeviceTemp,
                coralCANRangeDistance,
                coralDetected
        );

        final Notifier simUpdateNotifier = new Notifier(() -> {
            final double dt = deltaTime.get();
            motorSim.update(dt);
        });
        ToClose.add(simUpdateNotifier);
        simUpdateNotifier.setName(String.format(
                "SimUpdate(%d)",
                motor.getDeviceID()
        ));
        simUpdateNotifier.startPeriodic(SIM_UPDATE_PERIOD_SEC);
    }

    @Override
    public void config() {
        final CANrangeConfiguration CANRangeConfiguration = new CANrangeConfiguration();
        CANRangeConfiguration.ToFParams.UpdateMode = UpdateModeValue.ShortRange100Hz;
        CANRangeConfiguration.FovParams.FOVRangeX = 7;
        CANRangeConfiguration.FovParams.FOVRangeY = 7;
        CANRangeConfiguration.ProximityParams.ProximityThreshold = 0.1;
        CANRangeConfiguration.ProximityParams.ProximityHysteresis = 0.02;
        coralCANRange.getConfigurator().apply(CANRangeConfiguration);

        final TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();
        motorConfiguration.Slot0 = new Slot0Configs()
                .withKS(0.1)
                .withKV(0)
                .withKA(0.2)
                .withKP(50);
        motorConfiguration.CurrentLimits.StatorCurrentLimit = 40;
        motorConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        motorConfiguration.CurrentLimits.SupplyCurrentLimit = 40;
        motorConfiguration.CurrentLimits.SupplyCurrentLowerLimit = 40;
        motorConfiguration.CurrentLimits.SupplyCurrentLowerTime = 1;
        motorConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
        motorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motorConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        motorConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        motorConfiguration.Feedback.SensorToMechanismRatio = constants.wheelGearing();
        motor.getConfigurator().apply(motorConfiguration);

        BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                wheelPosition,
                wheelVelocity,
                wheelVoltage,
                wheelTorqueCurrent,
                coralCANRangeDistance,
                coralDetected
        );

        BaseStatusSignal.setUpdateFrequencyForAll(
                4,
                wheelDeviceTemp
        );

        ParentDevice.optimizeBusUtilizationForAll(
                4,
                motor,
                coralCANRange
        );

        motor.getSimState().Orientation = ChassisReference.CounterClockwise_Positive;
        coralCANRange.getSimState().setDistance(1);
    }

    @Override
    public void updateInputs(final GroundIntakeIO.GroundIntakeIOInputs inputs) {
        inputs.wheelPositionRots = wheelPosition.getValueAsDouble();
        inputs.wheelVelocityRotsPerSec = wheelVelocity.getValueAsDouble();
        inputs.wheelVoltage = wheelVoltage.getValueAsDouble();
        inputs.wheelTorqueCurrentAmps = wheelTorqueCurrent.getValueAsDouble();
        inputs.wheelTempCelsius = wheelDeviceTemp.getValueAsDouble();
        inputs.coralCANRangeDistanceMeters = coralCANRangeDistance.getValueAsDouble();
        inputs.coralDetected = coralDetected.getValue();
    }

    @Override
    public void toWheelVelocity(final double velocityRotsPerSec) {
        motor.setControl(velocityTorqueCurrentFOC.withVelocity(velocityRotsPerSec));
    }

    @Override
    public void toWheelVoltage(final double volts) {
        motor.setControl(voltageOut.withOutput(volts));
    }

    @Override
    public void toWheelTorqueCurrent(final double torqueCurrentAmps) {
        motor.setControl(torqueCurrentFOC.withOutput(torqueCurrentAmps));
    }

    @Override
    public void setCoralCANRangeDistance(final double distanceMeters) {
        coralCANRange.getSimState().setDistance(distanceMeters);
    }
}
