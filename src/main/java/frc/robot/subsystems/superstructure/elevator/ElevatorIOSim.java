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
import com.ctre.phoenix6.sim.ChassisReference;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.constants.HardwareConstants;
import frc.robot.constants.SimConstants;
import frc.robot.utils.closeables.ToClose;
import frc.robot.utils.control.DeltaTime;
import frc.robot.utils.ctre.Phoenix6Utils;
import frc.robot.utils.ctre.RefreshAll;
import frc.robot.utils.sim.motors.TalonFXSim;

import java.util.List;

public class ElevatorIOSim implements ElevatorIO {
    private static final double SIM_UPDATE_PERIOD_SEC = 0.005;

    private final DeltaTime deltaTime;
    private final HardwareConstants.ElevatorConstants constants;
    private final double drumCircumferenceMeters;

    private final ElevatorSim elevatorSim;

    private final TalonFX masterMotor;
    private final TalonFX followerMotor;
    private final TalonFXSim motorsSim;

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

    public ElevatorIOSim(final HardwareConstants.ElevatorConstants constants) {
        this.deltaTime = new DeltaTime(true);
        this.constants = constants;
        this.drumCircumferenceMeters = constants.spoolDiameterMeters() * Math.PI;

        final DCMotor dcMotors = DCMotor.getKrakenX60Foc(2);

        final double lowerLimitMeters = constants.lowerLimitRots() * drumCircumferenceMeters;
        final double upperLimitMeters = constants.upperLimitRots() * drumCircumferenceMeters;
        this.elevatorSim = new ElevatorSim(
                LinearSystemId.createElevatorSystem(
                        dcMotors,
                        SimConstants.Elevator.MASS_KG,
                        constants.spoolDiameterMeters() * 0.5,
                        constants.gearing()
                ),
                dcMotors,
                lowerLimitMeters,
                upperLimitMeters,
                true,
                lowerLimitMeters
        );

        this.masterMotor = new TalonFX(constants.rightMotorId(), constants.CANBus());
        this.followerMotor = new TalonFX(constants.leftMotorId(), constants.CANBus());

        this.motorsSim = new TalonFXSim(
                List.of(masterMotor, followerMotor),
                constants.gearing(),
                elevatorSim::update,
                elevatorSim::setInputVoltage,
                () -> Units.rotationsToRadians(elevatorSim.getPositionMeters() / drumCircumferenceMeters),
                () -> Units.rotationsToRadians(elevatorSim.getVelocityMetersPerSecond() / drumCircumferenceMeters)
        );

        this.motionMagicExpoVoltage = new MotionMagicExpoVoltage(0);
        this.dynamicMotionMagicVoltage = new DynamicMotionMagicVoltage(0, 0, 0, 0);
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

        final Notifier simUpdateNotifier = new Notifier(() -> {
            final double dt = deltaTime.get();
            motorsSim.update(dt);
        });
        ToClose.add(simUpdateNotifier);
        simUpdateNotifier.setName(String.format(
                "SimUpdate(%d,%d)",
                masterMotor.getDeviceID(),
                followerMotor.getDeviceID()
        ));
        simUpdateNotifier.startPeriodic(SIM_UPDATE_PERIOD_SEC);
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
                .withKP(32.42)
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
                masterMotor,
                followerMotor
        );

        masterMotor.getSimState().Orientation = ChassisReference.CounterClockwise_Positive;
        followerMotor.getSimState().Orientation = ChassisReference.CounterClockwise_Positive;
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
