package frc.robot.subsystems.superstructure.arm.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
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
import frc.robot.utils.sim.feedback.SimCANCoder;
import frc.robot.utils.sim.motors.TalonFXSim;

public class IntakeArmIOSim implements IntakeArmIO {
    private static final double SIM_UPDATE_PERIOD_SEC = 0.005;

    private final DeltaTime deltaTime;
    private final HardwareConstants.IntakeArmConstants constants;

    private final SingleJointedArmSim pivotSim;

    private final TalonFX pivotMotor;
    private final CANcoder pivotEncoder;
    private final TalonFXSim pivotTalonFXSim;

    private final PositionVoltage positionVoltage;
    private final TorqueCurrentFOC torqueCurrentFOC;
    private final VoltageOut voltageOut;

    private final StatusSignal<Angle> pivotPosition;
    private final StatusSignal<AngularVelocity> pivotVelocity;
    private final StatusSignal<Voltage> pivotVoltage;
    private final StatusSignal<Current> pivotTorqueCurrent;
    private final StatusSignal<Temperature> pivotDeviceTemp;
    private final StatusSignal<Angle> encoderPosition;
    private final StatusSignal<AngularVelocity> encoderVelocity;

    public IntakeArmIOSim(final HardwareConstants.IntakeArmConstants constants) {
        this.deltaTime = new DeltaTime(true);
        this.constants = constants;

        final double zeroedPositionToHorizontalRads = SimConstants.IntakeArm.ZEROED_POSITION_TO_HORIZONTAL.getRadians();
        this.pivotSim = new SingleJointedArmSim(
                LinearSystemId.identifyPositionSystem(
                        5.85 / (2d * Math.PI),
                        0.04 / (2d * Math.PI)
                ),
                MoreDCMotor.getMinion(1),
                constants.pivotGearing(),
                SimConstants.IntakeArm.LENGTH_METERS,
                Units.rotationsToRadians(constants.pivotLowerLimitRots()) + zeroedPositionToHorizontalRads,
                Units.rotationsToRadians(constants.pivotUpperLimitRots()) + zeroedPositionToHorizontalRads,
                true,
                SimConstants.IntakeArm.STARTING_ANGLE.getRadians()
        );

        this.pivotMotor = new TalonFX(constants.intakePivotMotorID(), constants.CANBus());
        this.pivotEncoder = new CANcoder(constants.intakePivotCANCoderId(), constants.CANBus());

        this.pivotTalonFXSim = new TalonFXSim(
                pivotMotor,
                constants.pivotGearing(),
                pivotSim::update,
                pivotSim::setInputVoltage,
                () -> pivotSim.getAngleRads() - zeroedPositionToHorizontalRads,
                pivotSim::getVelocityRadPerSec
        );
        this.pivotTalonFXSim.attachFeedbackSensor(new SimCANCoder(pivotEncoder));

        this.positionVoltage = new PositionVoltage(0);
        this.torqueCurrentFOC = new TorqueCurrentFOC(0);
        this.voltageOut = new VoltageOut(0);

        this.pivotPosition = pivotMotor.getPosition();
        this.pivotVelocity = pivotMotor.getVelocity();
        this.pivotVoltage = pivotMotor.getMotorVoltage();
        this.pivotTorqueCurrent = pivotMotor.getTorqueCurrent();
        this.pivotDeviceTemp = pivotMotor.getDeviceTemp();
        this.encoderPosition = pivotEncoder.getPosition();
        this.encoderVelocity = pivotEncoder.getVelocity();

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
        final CANcoderConfiguration pivotCANCoderConfiguration = new CANcoderConfiguration();
        pivotCANCoderConfiguration.MagnetSensor.MagnetOffset = constants.intakePivotCANCoderOffset();
        pivotCANCoderConfiguration.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        pivotEncoder.getConfigurator().apply(pivotCANCoderConfiguration);

        final TalonFXConfiguration pivotConfiguration = new TalonFXConfiguration();
        pivotConfiguration.Slot0 = new Slot0Configs()
                .withKG(0.26)
                .withGravityType(GravityTypeValue.Elevator_Static)
                .withKP(21);
        pivotConfiguration.TorqueCurrent.PeakForwardTorqueCurrent = 80;
        pivotConfiguration.TorqueCurrent.PeakReverseTorqueCurrent = -80;
        pivotConfiguration.CurrentLimits.StatorCurrentLimit = 60;
        pivotConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
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

        pivotMotor.getSimState().Orientation = ChassisReference.Clockwise_Positive;
        pivotEncoder.getSimState().Orientation = ChassisReference.Clockwise_Positive;
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
        pivotMotor.setControl(positionVoltage.withPosition(pivotPositionRots));
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
