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
import com.ctre.phoenix6.sim.ChassisReference;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.constants.HardwareConstants;
import frc.robot.constants.SimConstants;
import frc.robot.utils.MoreDCMotor;
import frc.robot.utils.closeables.ToClose;
import frc.robot.utils.control.DeltaTime;
import frc.robot.utils.ctre.RefreshAll;
import frc.robot.utils.sim.feedback.SimCANCoder;
import frc.robot.utils.sim.motors.TalonFXSim;

public class GroundIntakeArmIOSim implements GroundIntakeArmIO {
    private static final double SIM_UPDATE_PERIOD_SEC = 0.005;

    private final DeltaTime deltaTime;
    private final HardwareConstants.GroundIntakeArmConstants constants;

    private final SingleJointedArmSim pivotSim;

    private final TalonFX pivotMotor;
    private final CANcoder pivotEncoder;
    private final TalonFXSim pivotTalonFXSim;

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

    public GroundIntakeArmIOSim(final HardwareConstants.GroundIntakeArmConstants constants) {
        this.deltaTime = new DeltaTime(true);
        this.constants = constants;

        final double zeroedPositionToHorizontalRads = SimConstants.GroundIntakeArm
                .ZEROED_POSITION_TO_HORIZONTAL
                .getRadians();
        this.pivotSim = new SingleJointedArmSim(
                // TODO system gains
                LinearSystemId.identifyPositionSystem(
                        7.17 / (2d * Math.PI),
                        0.11 / (2d * Math.PI)
                ),
//                LinearSystemId.createSingleJointedArmSystem(
//                        MoreDCMotor.getKrakenX44(1),
//                        // TODO constants
//                        0.0416002666,
//                        constants.pivotGearing()
//                ),
                MoreDCMotor.getKrakenX44(1),
                constants.pivotGearing(),
                constants.lengthMeters(),
                Units.rotationsToRadians(constants.pivotLowerLimitRots()) - zeroedPositionToHorizontalRads,
                Units.rotationsToRadians(constants.pivotUpperLimitRots()) - zeroedPositionToHorizontalRads,
                true,
                SimConstants.GroundIntakeArm.STARTING_ANGLE.getRadians()
        );

        final HardwareConstants.CANBus bus = constants.CANBus();
        final CANBus p6Bus = bus.toPhoenix6CANBus();
        this.pivotMotor = new TalonFX(constants.pivotMotorID(), p6Bus);
        this.pivotEncoder = new CANcoder(constants.pivotCANCoderId(), p6Bus);

        this.pivotTalonFXSim = new TalonFXSim(
                pivotMotor,
                constants.pivotGearing(),
                pivotSim::update,
                pivotSim::setInputVoltage,
                () -> pivotSim.getAngleRads() + zeroedPositionToHorizontalRads,
                pivotSim::getVelocityRadPerSec
        );
        this.pivotTalonFXSim.attachFeedbackSensor(new SimCANCoder(pivotEncoder));

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

        final Notifier simUpdateNotifier = new Notifier(() -> {
            final double dt = deltaTime.get();
            pivotTalonFXSim.update(dt);
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
        final CANcoderConfiguration encoderConfiguration = new CANcoderConfiguration();
        encoderConfiguration.MagnetSensor.MagnetOffset = constants.pivotCANCoderOffset();
        encoderConfiguration.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        pivotEncoder.getConfigurator().apply(encoderConfiguration);

        // TODO gains + config
        final TalonFXConfiguration pivotMotorConfig = new TalonFXConfiguration();
        pivotMotorConfig.Slot0 = new Slot0Configs()
                .withKS(0)
                .withKG(0.3)
                .withGravityType(GravityTypeValue.Elevator_Static)
                .withKV(6.7)
                .withKA(0.06)
                .withKP(160)
                .withKD(3.3);
        pivotMotorConfig.MotionMagic.MotionMagicCruiseVelocity = 0;
        pivotMotorConfig.MotionMagic.MotionMagicExpo_kV = 14;
        pivotMotorConfig.MotionMagic.MotionMagicExpo_kA = 2.6;
        pivotMotorConfig.TorqueCurrent.PeakForwardTorqueCurrent = 60;
        pivotMotorConfig.TorqueCurrent.PeakReverseTorqueCurrent = -60;
        pivotMotorConfig.CurrentLimits.StatorCurrentLimit = 120;
        pivotMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        pivotMotorConfig.CurrentLimits.SupplyCurrentLimit = 70;
        pivotMotorConfig.CurrentLimits.SupplyCurrentLowerLimit = 55;
        pivotMotorConfig.CurrentLimits.SupplyCurrentLowerTime = 2.5;
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

        pivotMotor.getSimState().Orientation = ChassisReference.Clockwise_Positive;
        pivotEncoder.getSimState().Orientation = ChassisReference.Clockwise_Positive;
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
