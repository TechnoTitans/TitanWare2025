package frc.robot.subsystems.ground;

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
import edu.wpi.first.units.measure.*;
import frc.robot.constants.HardwareConstants;
import frc.robot.utils.ctre.RefreshAll;

public class GroundIntakeIOReal implements GroundIntakeIO {
    private final HardwareConstants.GroundIntakeConstants constants;

    private final TalonFX rollerMotor;
    private final CANrange coralCANRange;

    private final StatusSignal<Angle> rollerPosition;
    private final StatusSignal<AngularVelocity> rollerVelocity;
    private final StatusSignal<Voltage> rollerVoltage;
    private final StatusSignal<Current> rollerTorqueCurrent;
    private final StatusSignal<Temperature> rollerDeviceTemp;
    private final StatusSignal<Distance> rollerCANRangeDistance;

    private final VelocityTorqueCurrentFOC velocityTorqueCurrentFOC;
    private final TorqueCurrentFOC torqueCurrentFOC;
    private final VoltageOut voltageOut;

    public GroundIntakeIOReal(final HardwareConstants.GroundIntakeConstants constants) {
        this.constants = constants;

        this.rollerMotor = new TalonFX(constants.rollerMotorID(), constants.CANBus());
        this.coralCANRange = new CANrange(constants.coralTOFID(), constants.CANBus());

        this.rollerPosition = rollerMotor.getPosition(false);
        this.rollerVelocity = rollerMotor.getVelocity(false);
        this.rollerVoltage = rollerMotor.getMotorVoltage(false);
        this.rollerTorqueCurrent = rollerMotor.getTorqueCurrent(false);
        this.rollerDeviceTemp = rollerMotor.getDeviceTemp(false);
        this.rollerCANRangeDistance = coralCANRange.getDistance(false);

        this.velocityTorqueCurrentFOC = new VelocityTorqueCurrentFOC(0);
        this.torqueCurrentFOC = new TorqueCurrentFOC(0);
        this.voltageOut = new VoltageOut(0);

        RefreshAll.add(
                RefreshAll.CANBus.CANIVORE,
                rollerPosition,
                rollerVelocity,
                rollerVoltage,
                rollerTorqueCurrent,
                rollerDeviceTemp,
                rollerCANRangeDistance
        );
    }

    @Override
    public void config() {
        final CANrangeConfiguration CANRangeConfiguration = new CANrangeConfiguration();
        CANRangeConfiguration.ToFParams.UpdateMode = UpdateModeValue.ShortRange100Hz;
        CANRangeConfiguration.ToFParams.UpdateFrequency = 50;
        CANRangeConfiguration.FovParams.FOVRangeX = 7;
        CANRangeConfiguration.FovParams.FOVRangeY = 7;
        coralCANRange.getConfigurator().apply(CANRangeConfiguration);

        final TalonFXConfiguration rollerConfiguration = new TalonFXConfiguration();
        rollerConfiguration.Slot0 = new Slot0Configs()
//                .withKS(4.0212)
                .withKV(1);
//                .withKA(0.22711)
//                .withKP(100);
        rollerConfiguration.CurrentLimits.StatorCurrentLimit = 40;
        rollerConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        rollerConfiguration.CurrentLimits.SupplyCurrentLimit = 40;
        rollerConfiguration.CurrentLimits.SupplyCurrentLowerLimit = 40;
        rollerConfiguration.CurrentLimits.SupplyCurrentLowerTime = 1;
        rollerConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
        rollerConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        rollerConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        rollerConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        rollerConfiguration.Feedback.SensorToMechanismRatio = constants.rollerGearing();
        rollerMotor.getConfigurator().apply(rollerConfiguration);

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
    }

    @Override
    public void updateInputs(final GroundIntakeIO.GroundIntakeIOInputs inputs) {
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
    public void toRollerTorqueCurrent(double torqueCurrentAmps) {
        rollerMotor.setControl(torqueCurrentFOC.withOutput(torqueCurrentAmps));
    }

    @Override
    public void toRollerVoltage(double volts) {
        rollerMotor.setControl(voltageOut.withOutput(volts));
    }
}
