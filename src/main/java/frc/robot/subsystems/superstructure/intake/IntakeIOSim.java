package frc.robot.subsystems.superstructure.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
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
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.constants.HardwareConstants;
import frc.robot.constants.SimConstants;
import frc.robot.utils.MoreDCMotor;
import frc.robot.utils.closeables.ToClose;
import frc.robot.utils.control.DeltaTime;
import frc.robot.utils.sim.SimUtils;
import frc.robot.utils.sim.feedback.SimCANCoder;
import frc.robot.utils.sim.motors.TalonFXSim;

import java.util.concurrent.ThreadLocalRandom;

public class IntakeIOSim implements IntakeIO {
    private static final double SIM_UPDATE_PERIOD_SEC = 0.005;

    private final DeltaTime deltaTime;
    private final HardwareConstants.IntakeConstants constants;

    private final SingleJointedArmSim pivotSim;

    private final TalonFX pivotMotor;
    private final TalonFX coralRollerMotor;
    private final TalonFX algaeRollerMotor;
    private final CANcoder pivotEncoder;
    private final CANrange CANRange;

    private final TalonFXSim pivotTalonFXSim;
    private final TalonFXSim coralRollerTalonFXSim;
    private final TalonFXSim algaeRollerTalonFXSim;

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
    private final StatusSignal<Distance> CANRangeDistance;
    private final StatusSignal<Boolean> CANRangeIsDetected;

    public IntakeIOSim(final HardwareConstants.IntakeConstants constants) {
        this.deltaTime = new DeltaTime(true);
        this.constants = constants;

        this.pivotSim = new SingleJointedArmSim(
            LinearSystemId.identifyPositionSystem(
                    13.97 / (2d * Math.PI),
                    0.03 / (2d * Math.PI)
            ),
            MoreDCMotor.getMinion(1),
            constants.pivotGearing(),
            SimConstants.Intake.LENGTH_METERS,
            Units.rotationsToRadians(constants.pivotLowerLimitRots()),
            Math.PI,
            true,
            ThreadLocalRandom.current().nextDouble(
                    Units.rotationsToRadians(constants.pivotLowerLimitRots()),
                    Units.rotationsToRadians(constants.pivotUpperLimitRots())
            )
        );

        this.pivotMotor = new TalonFX(constants.intakePivotMotorID(), constants.CANBus());
        this.coralRollerMotor = new TalonFX(constants.coralRollerMotorID(), constants.CANBus());
        this.algaeRollerMotor = new TalonFX(constants.algaeRollerMotorID(), constants.CANBus());
        this.pivotEncoder = new CANcoder(constants.intakePivotCANCoderId(), constants.CANBus());
        this.CANRange = new CANrange(constants.CANRangeId(), constants.CANBus());

        this.pivotTalonFXSim = new TalonFXSim(
                pivotMotor,
                constants.pivotGearing(),
                pivotSim::update,
                pivotSim::setInputVoltage,
                pivotSim::getAngleRads,
                pivotSim::getVelocityRadPerSec
        );
        this.pivotTalonFXSim.attachFeedbackSensor(new SimCANCoder(pivotEncoder));

        final DCMotorSim coralRollerMotorSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                    0.19557 / (2 * Math.PI),
                    2.9856 / (2 * Math.PI)
            ),
            MoreDCMotor.getKrakenX44(1),
            constants.coralGearing()
        );
        this.coralRollerTalonFXSim = new TalonFXSim(
                coralRollerMotor,
                constants.coralGearing(),
                coralRollerMotorSim::update,
                coralRollerMotorSim::setInputVoltage,
                coralRollerMotorSim::getAngularPositionRad,
                coralRollerMotorSim::getAngularVelocityRadPerSec
        );

        final DCMotorSim algaeRollerMotorSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                    0.19557 / (2 * Math.PI),
                    2.9856 / (2 * Math.PI)
            ),
            MoreDCMotor.getKrakenX44(1),
            constants.algaeGearing()
        );
        this.algaeRollerTalonFXSim = new TalonFXSim(
                algaeRollerMotor,
                constants.algaeGearing(),
                algaeRollerMotorSim::update,
                algaeRollerMotorSim::setInputVoltage,
                algaeRollerMotorSim::getAngularPositionRad,
                algaeRollerMotorSim::getAngularVelocityRadPerSec
        );

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
        this.CANRangeDistance = CANRange.getDistance();
        this.CANRangeIsDetected = CANRange.getIsDetected();

        final Notifier simUpdateNotifier = new Notifier(() -> {
        final double dt = deltaTime.get();
            pivotTalonFXSim.update(dt);
            coralRollerTalonFXSim.update(dt);
            algaeRollerTalonFXSim.update(dt);
        });
        ToClose.add(simUpdateNotifier);
        simUpdateNotifier.setName(String.format(
                "SimUpdate(%d, %d, %d)",
                pivotMotor.getDeviceID(),
                coralRollerMotor.getDeviceID(),
                algaeRollerMotor.getDeviceID()
        ));
        simUpdateNotifier.startPeriodic(SIM_UPDATE_PERIOD_SEC);
    }

    @Override
    public void config() {
        final SensorDirectionValue pivotCANCoderDirection = SensorDirectionValue.Clockwise_Positive;
        final CANcoderConfiguration pivotCANCoderConfiguration = new CANcoderConfiguration();
        pivotCANCoderConfiguration.MagnetSensor.MagnetOffset = constants.intakePivotCANCoderOffset();
        pivotCANCoderConfiguration.MagnetSensor.SensorDirection = pivotCANCoderDirection;
        pivotEncoder.getConfigurator().apply(pivotCANCoderConfiguration);

        final InvertedValue pivotMotorInverted = InvertedValue.Clockwise_Positive;
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
        pivotConfiguration.MotorOutput.Inverted = pivotMotorInverted;
        pivotConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        pivotConfiguration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = constants.pivotUpperLimitRots();
        pivotConfiguration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        pivotConfiguration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = constants.pivotLowerLimitRots();
        pivotConfiguration.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        pivotMotor.getConfigurator().apply(pivotConfiguration);

        final InvertedValue coralRollerInvertedValue = InvertedValue.CounterClockwise_Positive;
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
        coralConfiguration.MotorOutput.Inverted = coralRollerInvertedValue;
        coralConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        coralConfiguration.Feedback.SensorToMechanismRatio = constants.coralGearing();
        coralRollerMotor.getConfigurator().apply(coralConfiguration);

        final InvertedValue algaeRollerInvertedValue = InvertedValue.CounterClockwise_Positive;
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
        algaeConfiguration.MotorOutput.Inverted = algaeRollerInvertedValue;
        algaeConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        algaeConfiguration.Feedback.SensorToMechanismRatio = constants.algaeGearing();
        algaeRollerMotor.getConfigurator().apply(algaeConfiguration);

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
                CANRangeDistance,
                CANRangeIsDetected
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
                CANRange
        );

        SimUtils.setCTRETalonFXSimStateMotorInverted(pivotMotor, pivotMotorInverted);
        SimUtils.setCTRETalonFXSimStateMotorInverted(coralRollerMotor, coralRollerInvertedValue);
        SimUtils.setCTRETalonFXSimStateMotorInverted(algaeRollerMotor, algaeRollerInvertedValue);
        SimUtils.setCTRETalonFXSimStateMotorInverted(pivotMotor, pivotMotorInverted);
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
                CANRangeDistance,
                CANRangeIsDetected,
                encoderPosition,
                encoderVelocity
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
        inputs.CANRangeDistanceMeters = CANRangeDistance.getValueAsDouble();
        inputs.CANRangeIsDetected = CANRangeIsDetected.getValue();
        inputs.encoderPositionRots = encoderPosition.getValueAsDouble();
        inputs.encoderVelocityRotsPerSec = encoderVelocity.getValueAsDouble();
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
