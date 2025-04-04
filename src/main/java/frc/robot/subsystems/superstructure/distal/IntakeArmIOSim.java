package frc.robot.subsystems.superstructure.distal;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFXS;
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
import frc.robot.utils.sim.motors.TalonFXSSim;

public class IntakeArmIOSim implements IntakeArmIO {
    private static final double SIM_UPDATE_PERIOD_SEC = 0.005;

    private final DeltaTime deltaTime;
    private final HardwareConstants.IntakeArmConstants constants;

    private final SingleJointedArmSim pivotSim;

    private final TalonFXS pivotMotor;
    private final CANcoder pivotEncoder;
    private final TalonFXSSim pivotTalonFXSSim;

    private final PositionVoltage positionVoltage;
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

        this.pivotMotor = new TalonFXS(constants.intakePivotMotorID(), constants.CANBus());
        this.pivotEncoder = new CANcoder(constants.intakePivotCANCoderId(), constants.CANBus());

        this.pivotTalonFXSSim = new TalonFXSSim(
                pivotMotor,
                constants.pivotGearing(),
                pivotSim::update,
                pivotSim::setInputVoltage,
                () -> pivotSim.getAngleRads() - zeroedPositionToHorizontalRads,
                pivotSim::getVelocityRadPerSec
        );
        this.pivotTalonFXSSim.attachFeedbackSensor(new SimCANCoder(pivotEncoder));

        this.positionVoltage = new PositionVoltage(0);
        this.voltageOut = new VoltageOut(0);

        this.pivotPosition = pivotMotor.getPosition();
        this.pivotVelocity = pivotMotor.getVelocity();
        this.pivotVoltage = pivotMotor.getMotorVoltage();
        this.pivotTorqueCurrent = pivotMotor.getTorqueCurrent();
        this.pivotDeviceTemp = pivotMotor.getDeviceTemp();
        this.encoderPosition = pivotEncoder.getPosition();
        this.encoderVelocity = pivotEncoder.getVelocity();

        RefreshAll.add(
                RefreshAll.CANBus.RIO,
                pivotPosition,
                pivotVelocity,
                pivotVoltage,
                pivotTorqueCurrent,
                pivotDeviceTemp,
                encoderPosition,
                encoderVelocity
        );

        final Notifier simUpdateNotifier = new Notifier(() -> {
            final double dt = deltaTime.get();
            pivotTalonFXSSim.update(dt);
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
        pivotCANCoderConfiguration.MagnetSensor.MagnetOffset = 0;
        pivotCANCoderConfiguration.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        pivotEncoder.getConfigurator().apply(pivotCANCoderConfiguration);

        final TalonFXSConfiguration pivotConfiguration = new TalonFXSConfiguration();
        pivotConfiguration.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
        pivotConfiguration.Commutation.AdvancedHallSupport = AdvancedHallSupportValue.Enabled;
        pivotConfiguration.Slot0 = new Slot0Configs()
                .withKG(0.26)
                .withGravityType(GravityTypeValue.Elevator_Static)
                .withKP(26);
        pivotConfiguration.CurrentLimits.StatorCurrentLimit = 60;
        pivotConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        pivotConfiguration.CurrentLimits.SupplyCurrentLimit = 50;
        pivotConfiguration.CurrentLimits.SupplyCurrentLowerLimit = 40;
        pivotConfiguration.CurrentLimits.SupplyCurrentLowerTime = 1;
        pivotConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
        pivotConfiguration.ExternalFeedback.ExternalFeedbackSensorSource = ExternalFeedbackSensorSourceValue.FusedCANcoder;
        pivotConfiguration.ExternalFeedback.FeedbackRemoteSensorID = pivotEncoder.getDeviceID();
        pivotConfiguration.ExternalFeedback.RotorToSensorRatio = constants.pivotGearing();
        pivotConfiguration.ExternalFeedback.SensorToMechanismRatio = 1;
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

        pivotMotor.getSimState().MotorOrientation = ChassisReference.Clockwise_Positive;
        pivotMotor.getSimState().ExtSensorOrientation = ChassisReference.Clockwise_Positive;
        pivotEncoder.getSimState().Orientation = ChassisReference.Clockwise_Positive;
    }

    @SuppressWarnings("DuplicatedCode")
    @Override
    public void updateInputs(IntakeArmIOInputs inputs) {
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
