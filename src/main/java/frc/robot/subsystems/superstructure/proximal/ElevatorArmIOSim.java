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
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.sim.ChassisReference;
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
import frc.robot.utils.ctre.RefreshAll;
import frc.robot.utils.sim.feedback.SimCANCoder;
import frc.robot.utils.sim.motors.TalonFXSim;

public class ElevatorArmIOSim implements ElevatorArmIO {
    private static final double SIM_UPDATE_PERIOD_SEC = 0.005;

    private final DeltaTime deltaTime;
    private final HardwareConstants.ElevatorArmConstants constants;

    private final SingleJointedArmSim pivotSim;

    private final TalonFX pivotMotor;
    private final CANcoder pivotCANCoder;
    private final TalonFXSim pivotMotorSim;

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

    public ElevatorArmIOSim(final HardwareConstants.ElevatorArmConstants constants) {
        this.deltaTime = new DeltaTime(true);
        this.constants = constants;

        final DCMotor dcMotor = DCMotor.getKrakenX60Foc(1);

        final double zeroedPositionToHorizontalRads =
                SimConstants.ElevatorArm.ZEROED_POSITION_TO_HORIZONTAL.getRadians();
        this.pivotSim = new SingleJointedArmSim(
                LinearSystemId.createSingleJointedArmSystem(
                        dcMotor,
                        //TODO: This is only correct when retracted
                        SimConstants.ElevatorArm.RETRACTED_MOI_KG_M_SQUARED,
                        constants.gearing()
                ),
                dcMotor,
                constants.gearing(),
                SimConstants.ElevatorArm.LENGTH_METERS,
                Units.rotationsToRadians(constants.lowerLimitRots()) - zeroedPositionToHorizontalRads,
                Units.rotationsToRadians(constants.upperLimitRots()) - zeroedPositionToHorizontalRads,
                false,
                SimConstants.ElevatorArm.STARTING_ANGLE.getRadians()
        );

        this.pivotMotor = new TalonFX(constants.motorId(), constants.CANBus());
        this.pivotCANCoder = new CANcoder(constants.CANCoderId(), constants.CANBus());

        this.pivotMotorSim = new TalonFXSim(
                pivotMotor,
                constants.gearing(),
                pivotSim::update,
                pivotSim::setInputVoltage,
                () -> pivotSim.getAngleRads() - zeroedPositionToHorizontalRads,
                pivotSim::getVelocityRadPerSec
        );
        this.pivotMotorSim.attachFeedbackSensor(new SimCANCoder(pivotCANCoder));

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
        final CANcoderConfiguration pivotCANCoderConfig = new CANcoderConfiguration();
        pivotCANCoderConfig.MagnetSensor.MagnetOffset = 0;
        pivotCANCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        pivotCANCoder.getConfigurator().apply(pivotCANCoderConfig);

        final TalonFXConfiguration pivotMotorConfig = new TalonFXConfiguration();
        //TODO: Get real constants from robot
        pivotMotorConfig.Slot0 = new Slot0Configs()
                .withKP(500);
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
                pivotMotor,
                pivotCANCoder
        );

        pivotCANCoder.getSimState().Orientation = ChassisReference.CounterClockwise_Positive;
        pivotMotor.getSimState().Orientation = ChassisReference.Clockwise_Positive;
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
