package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
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

public class IntakeIOSim implements IntakeIO {
    private static final double SIM_UPDATE_PERIOD_SEC = 0.005;

    private final DeltaTime deltaTime;
    private final HardwareConstants.IntakeConstants constants;

    private final TalonFX rollerMotor;
    private final CANrange coralCANRange;

    private final TalonFXSim rollerTalonFXSim;

    private final VelocityTorqueCurrentFOC velocityTorqueCurrentFOC;
    private final TorqueCurrentFOC torqueCurrentFOC;
    private final VoltageOut voltageOut;

    private final StatusSignal<Angle> rollerPosition;
    private final StatusSignal<AngularVelocity> rollerVelocity;
    private final StatusSignal<Voltage> rollerVoltage;
    private final StatusSignal<Current> rollerTorqueCurrent;
    private final StatusSignal<Temperature> rollerDeviceTemp;
    private final StatusSignal<Distance> rollerCANRangeDistance;

    public IntakeIOSim(final HardwareConstants.IntakeConstants constants) {
        this.deltaTime = new DeltaTime(true);
        this.constants = constants;

        this.rollerMotor = new TalonFX(constants.rollerRollerMotorID(), constants.CANBus());
        this.coralCANRange = new CANrange(constants.coralTOFID(), constants.CANBus());

        final DCMotorSim rollerMotorSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                    0.19557 / (2 * Math.PI),
                    2.9856 / (2 * Math.PI)
            ),
            MoreDCMotor.getKrakenX44(1)
        );
        this.rollerTalonFXSim = new TalonFXSim(
                rollerMotor,
                constants.rollerGearing(),
                rollerMotorSim::update,
                rollerMotorSim::setInputVoltage,
                rollerMotorSim::getAngularPositionRad,
                rollerMotorSim::getAngularVelocityRadPerSec
        );

        this.velocityTorqueCurrentFOC = new VelocityTorqueCurrentFOC(0);
        this.torqueCurrentFOC = new TorqueCurrentFOC(0);
        this.voltageOut = new VoltageOut(0);

        this.rollerPosition = rollerMotor.getPosition();
        this.rollerVelocity = rollerMotor.getVelocity();
        this.rollerVoltage = rollerMotor.getMotorVoltage();
        this.rollerTorqueCurrent = rollerMotor.getTorqueCurrent();
        this.rollerDeviceTemp = rollerMotor.getDeviceTemp();
        this.rollerCANRangeDistance = coralCANRange.getDistance();

        RefreshAll.add(
                RefreshAll.CANBus.RIO,
                rollerPosition,
                rollerVelocity,
                rollerVoltage,
                rollerTorqueCurrent,
                rollerDeviceTemp,
                rollerCANRangeDistance
        );

        final Notifier simUpdateNotifier = new Notifier(() -> {
        final double dt = deltaTime.get();
            rollerTalonFXSim.update(dt);
        });
        ToClose.add(simUpdateNotifier);
        simUpdateNotifier.setName(String.format(
                "SimUpdate(%d)",
                rollerMotor.getDeviceID()
        ));
        simUpdateNotifier.startPeriodic(SIM_UPDATE_PERIOD_SEC);
    }

    @Override
    public void config() {
        final CANrangeConfiguration CANRangeConfiguration = new CANrangeConfiguration();
        CANRangeConfiguration.ToFParams.UpdateMode = UpdateModeValue.LongRangeUserFreq;
        CANRangeConfiguration.ToFParams.UpdateFrequency = 50;
        CANRangeConfiguration.FovParams.FOVRangeX = 7;
        CANRangeConfiguration.FovParams.FOVRangeY = 7;
        coralCANRange.getConfigurator().apply(CANRangeConfiguration);

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
        coralConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        coralConfiguration.MotorOutput.Inverted = coralRollerInvertedValue;
        coralConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        coralConfiguration.Feedback.SensorToMechanismRatio = constants.rollerGearing();
        rollerMotor.getConfigurator().apply(coralConfiguration);

        BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                rollerPosition,
                rollerVelocity,
                rollerVoltage,
                rollerTorqueCurrent,
                rollerCANRangeDistance
        );

        BaseStatusSignal.setUpdateFrequencyForAll(
                4,
                rollerDeviceTemp
        );

        ParentDevice.optimizeBusUtilizationForAll(
                4,
                rollerMotor,
                coralCANRange
        );

        rollerMotor.getSimState().Orientation = ChassisReference.CounterClockwise_Positive;
        coralCANRange.getSimState().setDistance(10);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.rollerPositionRots = rollerPosition.getValueAsDouble();
        inputs.rollerVelocityRotsPerSec = rollerVelocity.getValueAsDouble();
        inputs.rollerVoltage = rollerVoltage.getValueAsDouble();
        inputs.rollerTorqueCurrentAmps = rollerTorqueCurrent.getValueAsDouble();
        inputs.rollerTempCelsius = rollerDeviceTemp.getValueAsDouble();
        inputs.coralTOFDistanceMeters = rollerCANRangeDistance.getValueAsDouble();
    }

    @Override
    public void toRollerVelocity(final double velocityRotsPerSec) {
        rollerMotor.setControl(velocityTorqueCurrentFOC.withVelocity(velocityRotsPerSec));
    }

    @Override
    public void toRollerVoltage(final double volts) {
        rollerMotor.setControl(voltageOut.withOutput(volts));
    }

    @Override
    public void toRollerTorqueCurrent(final double torqueCurrentAmps) {
        rollerMotor.setControl(torqueCurrentFOC.withOutput(torqueCurrentAmps));
    }

    @Override
    public void setTOFDistance(final double distanceMeters) {
        coralCANRange.getSimState().setDistance(distanceMeters);
    }
}
