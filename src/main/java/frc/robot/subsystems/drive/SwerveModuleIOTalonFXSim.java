package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.util.DoubleCircularBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.constants.SimConstants;
import frc.robot.subsystems.drive.constants.SwerveConstants;
import frc.robot.utils.closeables.ToClose;
import frc.robot.utils.control.DeltaTime;
import frc.robot.utils.ctre.Phoenix6Utils;
import frc.robot.utils.sim.SimUtils;
import frc.robot.utils.sim.feedback.SimCANCoder;
import frc.robot.utils.sim.motors.TalonFXSim;

import static frc.robot.subsystems.drive.constants.SwerveConstants.Config;

public class SwerveModuleIOTalonFXSim implements SwerveModuleIO {
    private static final double SIM_UPDATE_PERIOD_SEC = 0.005;

    private final TalonFX driveMotor;
    private final TalonFX turnMotor;
    private final TalonFXSim driveSim;
    private final TalonFXSim turnSim;
    private final CANcoder turnEncoder;
    private final double magnetOffset;

    private final double driveReduction = Config.driveReduction();
    private final double turnReduction = Config.turnReduction();
    private final double couplingRatio = Config.couplingRatio();

    private final TalonFXConfiguration driveTalonFXConfiguration = new TalonFXConfiguration();
    private final TalonFXConfiguration turnTalonFXConfiguration = new TalonFXConfiguration();

    private final VelocityTorqueCurrentFOC velocityTorqueCurrentFOC;
    private final PositionVoltage positionVoltage;

    private final OdometryThreadRunner odometryThreadRunner;

    private final DeltaTime deltaTime;

    // Cached StatusSignals
    private final StatusSignal<Angle> drivePosition;
    private final StatusSignal<AngularVelocity> driveVelocity;
    private final StatusSignal<Current> driveTorqueCurrent;
    private final StatusSignal<Temperature> driveDeviceTemp;
    private final StatusSignal<Angle> turnPosition;
    private final StatusSignal<AngularVelocity> turnVelocity;
    private final StatusSignal<Current> turnTorqueCurrent;
    private final StatusSignal<Temperature> turnDeviceTemp;

    // Odometry StatusSignal update buffers
    private final DoubleCircularBuffer timestampBuffer;
    private final DoubleCircularBuffer drivePositionSignalBuffer;
    private final DoubleCircularBuffer turnPositionSignalBuffer;

    public SwerveModuleIOTalonFXSim(
            final SwerveConstants.SwerveModuleConstants constants,
            final OdometryThreadRunner odometryThreadRunner
    ) {
        this.driveMotor = new TalonFX(constants.driveMotorId(), constants.moduleCANBus());
        this.turnMotor = new TalonFX(constants.turnMotorId(), constants.moduleCANBus());

        final DCMotor driveDCMotor = DCMotor.getKrakenX60Foc(1);
        final DCMotorSim driveDCMotorSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                        driveDCMotor,
                        SimConstants.SwerveModules.DRIVE_WHEEL_MOMENT_OF_INERTIA_KG_M_SQUARED,
                        driveReduction
                ),
                driveDCMotor
        );

        this.driveSim = new TalonFXSim(
                driveMotor,
                driveReduction,
                driveDCMotorSim::update,
                (motorVoltage) -> driveDCMotorSim.setInputVoltage(
                        SimUtils.addMotorFriction(motorVoltage, SimConstants.SwerveModules.DRIVE_KS_VOLTS)
                ),
                driveDCMotorSim::getAngularPositionRad,
                driveDCMotorSim::getAngularVelocityRadPerSec
        );

        final DCMotor turnDCMotor = DCMotor.getFalcon500Foc(1);
        final DCMotorSim turnDCMotorSim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(
                        turnDCMotor,
                        SimConstants.SwerveModules.TURN_WHEEL_MOMENT_OF_INERTIA_KG_M_SQUARED,
                        turnReduction
                ),
                turnDCMotor
        );

        this.turnEncoder = new CANcoder(constants.turnEncoderId(), constants.moduleCANBus());
        this.magnetOffset = constants.turnEncoderOffsetRots();
        this.turnSim = new TalonFXSim(
                turnMotor,
                turnReduction,
                turnDCMotorSim::update,
                (motorVoltage) -> turnDCMotorSim.setInputVoltage(
                        SimUtils.addMotorFriction(motorVoltage, SimConstants.SwerveModules.STEER_KS_VOLTS)
                ),
                turnDCMotorSim::getAngularPositionRad,
                turnDCMotorSim::getAngularVelocityRadPerSec
        );
        this.turnSim.attachFeedbackSensor(new SimCANCoder(turnEncoder));

        this.velocityTorqueCurrentFOC = new VelocityTorqueCurrentFOC(0);
        this.positionVoltage = new PositionVoltage(0);
        this.odometryThreadRunner = odometryThreadRunner;
        this.odometryThreadRunner.registerControlRequest(driveMotor, velocityTorqueCurrentFOC, driveMotor::setControl);
        this.odometryThreadRunner.registerControlRequest(turnMotor, positionVoltage, turnMotor::setControl);

        this.deltaTime = new DeltaTime(true);

        this.drivePosition = driveMotor.getPosition();
        this.driveVelocity = driveMotor.getVelocity();
        this.driveTorqueCurrent = driveMotor.getTorqueCurrent();
        this.driveDeviceTemp = driveMotor.getDeviceTemp();
        this.turnPosition = turnEncoder.getAbsolutePosition();
        this.turnVelocity = turnEncoder.getVelocity();
        this.turnTorqueCurrent = turnMotor.getTorqueCurrent();
        this.turnDeviceTemp = turnMotor.getDeviceTemp();

        this.timestampBuffer = odometryThreadRunner.makeTimestampBuffer();
        this.drivePositionSignalBuffer = odometryThreadRunner.registerSignal(driveMotor, this.drivePosition);
        this.turnPositionSignalBuffer = odometryThreadRunner.registerSignal(turnMotor, this.turnPosition);

        final Notifier simUpdateNotifier = new Notifier(() -> {
            final double dtSeconds = deltaTime.get();
            driveSim.update(dtSeconds);
            turnSim.update(dtSeconds);
        });
        ToClose.add(simUpdateNotifier);
        simUpdateNotifier.setName(String.format(
                "SimUpdate(%d,%d)",
                driveMotor.getDeviceID(),
                turnMotor.getDeviceID()
        ));
        simUpdateNotifier.startPeriodic(SIM_UPDATE_PERIOD_SEC);
    }

    @SuppressWarnings("DuplicatedCode")
    @Override
    public void config() {
        final CANcoderConfiguration canCoderConfiguration = new CANcoderConfiguration();
        canCoderConfiguration.MagnetSensor.MagnetOffset = -magnetOffset;
        canCoderConfiguration.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        turnEncoder.getConfigurator().apply(canCoderConfiguration);

        // TODO: I think we need to look at VoltageConfigs and/or CurrentLimitConfigs for limiting the
        //  current we can apply in sim, this is cause we use VelocityVoltage in sim instead of VelocityTorqueCurrentFOC
        //  which means that TorqueCurrent.PeakForwardTorqueCurrent and related won't affect it
        final InvertedValue driveInvertedValue = InvertedValue.Clockwise_Positive;
        driveTalonFXConfiguration.Slot0 = new Slot0Configs()
                .withKP(50)
                .withKS(4.796)
                .withKA(2.549);
        driveTalonFXConfiguration.TorqueCurrent.PeakForwardTorqueCurrent = 80;
        driveTalonFXConfiguration.TorqueCurrent.PeakReverseTorqueCurrent = -80;
        driveTalonFXConfiguration.CurrentLimits.StatorCurrentLimit = 80;
        driveTalonFXConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        driveTalonFXConfiguration.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.2;
        driveTalonFXConfiguration.Feedback.SensorToMechanismRatio = driveReduction;
        driveTalonFXConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        driveTalonFXConfiguration.MotorOutput.Inverted = driveInvertedValue;
        driveMotor.getConfigurator().apply(driveTalonFXConfiguration);

        final InvertedValue turnInvertedValue = InvertedValue.Clockwise_Positive;
        turnTalonFXConfiguration.Slot0 = new Slot0Configs()
                .withKP(30)
                .withKS(0.5);
        turnTalonFXConfiguration.TorqueCurrent.PeakForwardTorqueCurrent = 40;
        turnTalonFXConfiguration.TorqueCurrent.PeakReverseTorqueCurrent = -40;
        turnTalonFXConfiguration.Feedback.FeedbackRemoteSensorID = turnEncoder.getDeviceID();
        turnTalonFXConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        turnTalonFXConfiguration.Feedback.RotorToSensorRatio = turnReduction;
        turnTalonFXConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        turnTalonFXConfiguration.MotorOutput.Inverted = turnInvertedValue;
        turnMotor.getConfigurator().apply(turnTalonFXConfiguration);

        velocityTorqueCurrentFOC.UpdateFreqHz = 0;
        positionVoltage.UpdateFreqHz = 0;

        // TODO: this fix for CANCoder initialization in sim doesn't seem to work all the time...investigate!
//      SimUtils.initializeCTRECANCoderSim(turnEncoder);
        SimUtils.setCTRETalonFXSimStateMotorInverted(driveMotor, driveInvertedValue);
        SimUtils.setCTRETalonFXSimStateMotorInverted(turnMotor, turnInvertedValue);
    }

    @SuppressWarnings("DuplicatedCode")
    @Override
    public void updateInputs(final SwerveModuleIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                this.drivePosition,
                this.driveVelocity,
                this.driveTorqueCurrent,
                this.driveDeviceTemp,
                this.turnPosition,
                this.turnVelocity,
                this.turnTorqueCurrent,
                this.turnDeviceTemp
        );

        inputs.drivePositionRots = getDrivePosition();
        inputs.driveVelocityRotsPerSec = this.driveVelocity.getValueAsDouble();
        inputs.driveTorqueCurrentAmps = this.driveTorqueCurrent.getValueAsDouble();
        inputs.driveTempCelsius = this.driveDeviceTemp.getValueAsDouble();

        inputs.turnPositionRots = getRawAngle();
        inputs.turnVelocityRotsPerSec = this.turnVelocity.getValueAsDouble();
        inputs.turnTorqueCurrentAmps = this.turnTorqueCurrent.getValueAsDouble();
        inputs.turnTempCelsius = this.turnDeviceTemp.getValueAsDouble();

        inputs.odometryTimestampsSec = OdometryThreadRunner.writeBufferToArray(timestampBuffer);
        timestampBuffer.clear();

        inputs.odometryDrivePositionsRots = OdometryThreadRunner.writeBufferToArray(drivePositionSignalBuffer);
        drivePositionSignalBuffer.clear();

        inputs.odometryTurnPositionRots = OdometryThreadRunner.writeBufferToArray(turnPositionSignalBuffer);
        turnPositionSignalBuffer.clear();
    }

    /**
     * Get the measured mechanism (wheel) angle of the {@link SwerveModuleIOTalonFXSim}, in raw units (rotations)
     *
     * @return the measured wheel (turner) angle, in rotations
     */
    private double getRawAngle() {
        return Phoenix6Utils.latencyCompensateIfSignalIsGood(this.turnPosition, this.turnVelocity);
    }

    public double getDrivePosition() {
        final double driveWheelPosition = Phoenix6Utils.latencyCompensateIfSignalIsGood(
                this.drivePosition, this.driveVelocity
        );
        final double turnPosition = Phoenix6Utils.latencyCompensateIfSignalIsGood(
                this.turnPosition, this.turnVelocity
        );
        final double driveBackOutWheelRotations = (
                (turnPosition * couplingRatio)
                        / driveReduction
        );

        return driveWheelPosition - driveBackOutWheelRotations;
    }

    @SuppressWarnings("DuplicatedCode")
    @Override
    public void setInputs(
            final double desiredDriverVelocity,
            final double desiredTurnerRotations,
            final double feedforwardAmps
    ) {
        final double driveVelocityBackOut = (
                (this.turnVelocity.getValueAsDouble() * couplingRatio)
                        / driveReduction
        );
        final double backedOutDriveVelocity = desiredDriverVelocity + driveVelocityBackOut;

        odometryThreadRunner.updateControlRequest(driveMotor, velocityTorqueCurrentFOC);
        driveMotor.setControl(velocityTorqueCurrentFOC
                .withVelocity(backedOutDriveVelocity)
                .withFeedForward(feedforwardAmps)
        );
        turnMotor.setControl(positionVoltage.withPosition(desiredTurnerRotations));
    }

    @Override
    public void setNeutralMode(final NeutralModeValue neutralMode) {
        if (SimConstants.CTRE.DISABLE_NEUTRAL_MODE_IN_SIM) {
            // just ignore setNeutralMode call if NeutralMode setting in Simulation is turned off
            return;
        }

        final StatusCode refreshCode = driveMotor.getConfigurator()
                .refresh(turnTalonFXConfiguration, SimConstants.CTRE.CONFIG_TIMEOUT_SECONDS);
        if (!refreshCode.isOK()) {
            // warn if the refresh call failed in sim, which might happen pretty often as
            // there seems to be an issue with calling refresh while disabled in sim
            DriverStation.reportWarning(
                    String.format(
                            "Failed to set NeutralMode on TalonFX %s (%s)",
                            driveMotor.getDeviceID(),
                            driveMotor.getNetwork()
                    ), false
            );
            return;
        }

        turnTalonFXConfiguration.MotorOutput.NeutralMode = neutralMode;
        driveMotor.getConfigurator().apply(turnTalonFXConfiguration);
    }
}
