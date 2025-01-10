package frc.robot.subsystems.superstructure.arm;

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
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.constants.HardwareConstants;
import frc.robot.constants.SimConstants;
import frc.robot.utils.closeables.ToClose;
import frc.robot.utils.control.DeltaTime;
import frc.robot.utils.sim.SimUtils;
import frc.robot.utils.sim.feedback.SimCANCoder;
import frc.robot.utils.sim.motors.TalonFXSim;

import java.util.concurrent.ThreadLocalRandom;

public class ArmIOSim implements ArmIO {
    private static final double SIM_UPDATE_PERIOD_SEC = 0.005;

    private final DeltaTime deltaTime;
    private final HardwareConstants.ArmConstants constants;

    private final SingleJointedArmSim pivotSim;

    private final TalonFX pivotMotor;
    private final CANcoder pivotCANCoder;
    private final TalonFXSim pivotMotorSim;

    private final MotionMagicExpoTorqueCurrentFOC motionMagicExpoTorqueCurrentFOC;
    private final TorqueCurrentFOC torqueCurrentFOC;
    private final VoltageOut voltageOut;

    private final StatusSignal<Angle> pivotPosition;
    private final StatusSignal<AngularVelocity> pivotVelocity;
    private final StatusSignal<Voltage> pivotVoltage;
    private final StatusSignal<Current> pivotTorqueCurrent;
    private final StatusSignal<Temperature> pivotDeviceTemp;
    private final StatusSignal<Angle> pivotCANCoderPosition;
    private final StatusSignal<AngularVelocity> pivotCANCoderVelocity;

    public ArmIOSim(final HardwareConstants.ArmConstants constants) {
        this.deltaTime = new DeltaTime(true);
        this.constants = constants;

        this.pivotSim = new SingleJointedArmSim(
                LinearSystemId.identifyPositionSystem(
                        13.97 / (2d * Math.PI),
                        0.03 / (2d * Math.PI)
                ),
                DCMotor.getKrakenX60(1),
                constants.gearing(),
                SimConstants.Arm.LENGTH_METERS,
                Units.rotationsToRadians(constants.lowerLimitRots()),
                Math.PI,
                true,
                ThreadLocalRandom.current().nextDouble(
                        Units.rotationsToRadians(constants.lowerLimitRots()),
                        Units.rotationsToRadians(constants.upperLimitRots())
                )
        );

        this.pivotMotor = new TalonFX(constants.motorId(), constants.CANBus());
        this.pivotCANCoder = new CANcoder(constants.CANCoderId(), constants.CANBus());

        this.pivotMotorSim = new TalonFXSim(
                pivotMotor,
                constants.gearing(),
                pivotSim::update,
                pivotSim::setInputVoltage,
                pivotSim::getAngleRads,
                pivotSim::getVelocityRadPerSec
        );
        this.pivotMotorSim.attachFeedbackSensor(new SimCANCoder(pivotCANCoder));

        this.motionMagicExpoTorqueCurrentFOC = new MotionMagicExpoTorqueCurrentFOC(0);
        this.torqueCurrentFOC = new TorqueCurrentFOC(0);
        this.voltageOut = new VoltageOut(0);

        this.pivotPosition = pivotMotor.getPosition();
        this.pivotVelocity = pivotMotor.getVelocity();
        this.pivotVoltage = pivotMotor.getMotorVoltage();
        this.pivotTorqueCurrent = pivotMotor.getTorqueCurrent();
        this.pivotDeviceTemp = pivotMotor.getDeviceTemp();
        this.pivotCANCoderPosition = pivotCANCoder.getPosition();
        this.pivotCANCoderVelocity = pivotCANCoder.getVelocity();

        final Notifier simUpdateNotifier = new Notifier(() -> {
        final double dt = deltaTime.get();
            pivotMotorSim.update(dt);
        });
        ToClose.add(simUpdateNotifier);
        simUpdateNotifier.setName(String.format(
                "SimUpdate(%d)",
                pivotMotor.getDeviceID()
        ));
        simUpdateNotifier.startPeriodic(SIM_UPDATE_PERIOD_SEC);
    }

    @Override
    public void config() {
        final SensorDirectionValue pivotCANCoderSensorDirection = SensorDirectionValue.Clockwise_Positive;
        final CANcoderConfiguration pivotCANCoderConfig = new CANcoderConfiguration();
        pivotCANCoderConfig.MagnetSensor.MagnetOffset = constants.CANCoderOffset();
        pivotCANCoderConfig.MagnetSensor.SensorDirection = pivotCANCoderSensorDirection;
        pivotCANCoder.getConfigurator().apply(pivotCANCoderConfig);

        final InvertedValue pivotMotorInverted = InvertedValue.Clockwise_Positive;
        final TalonFXConfiguration pivotMotorConfig = new TalonFXConfiguration();
        pivotMotorConfig.Slot0 = new Slot0Configs()
                .withKS(0)
                .withKG(0.11)
                .withGravityType(GravityTypeValue.Arm_Cosine)
                .withKV(13.97)
                .withKA(0.015)
                .withKP(50);
        pivotMotorConfig.MotionMagic.MotionMagicCruiseVelocity = 0;
        pivotMotorConfig.MotionMagic.MotionMagicExpo_kV = 13.97;
        pivotMotorConfig.MotionMagic.MotionMagicExpo_kA = 0.015;
        pivotMotorConfig.TorqueCurrent.PeakForwardTorqueCurrent = 80;
        pivotMotorConfig.TorqueCurrent.PeakReverseTorqueCurrent = -80;
        pivotMotorConfig.CurrentLimits.StatorCurrentLimit = 60;
        pivotMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        pivotMotorConfig.CurrentLimits.SupplyCurrentLimit = 50;
        pivotMotorConfig.CurrentLimits.SupplyCurrentLowerLimit = 40;
        pivotMotorConfig.CurrentLimits.SupplyCurrentLowerTime = 1;
        pivotMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        pivotMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        pivotMotorConfig.Feedback.FeedbackRemoteSensorID = pivotCANCoder.getDeviceID();
        pivotMotorConfig.Feedback.RotorToSensorRatio = constants.gearing();
        pivotMotorConfig.Feedback.SensorToMechanismRatio = 1;
        pivotMotorConfig.MotorOutput.Inverted = pivotMotorInverted;
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
                pivotMotor,
                pivotCANCoder
        );

        SimUtils.setCTRETalonFXSimStateMotorInverted(pivotMotor, pivotMotorInverted);
        SimUtils.setCTRECANCoderSimStateSensorDirection(pivotCANCoder, pivotCANCoderSensorDirection);
    }

    @Override
    public void updateInputs(final ArmIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                pivotPosition,
                pivotVelocity,
                pivotVoltage,
                pivotTorqueCurrent,
                pivotCANCoderPosition,
                pivotCANCoderVelocity
        );

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
        pivotMotor.setControl(motionMagicExpoTorqueCurrentFOC.withPosition(pivotPositionRots));
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
