package frc.robot.subsystems.gyro;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.util.DoubleCircularBuffer;
import frc.robot.constants.HardwareConstants;
import frc.robot.subsystems.drive.OdometryThreadRunner;
import frc.robot.utils.ctre.Phoenix6Utils;
import frc.robot.utils.ctre.RefreshAll;

public class GyroIOPigeon2 implements GyroIO {
    private final Pigeon2 pigeon;

    // Cached StatusSignals
    private final StatusSignal<Angle> yaw;
    private final StatusSignal<Angle> pitch;
    private final StatusSignal<Angle> roll;
    private final StatusSignal<AngularVelocity> yawVelocity;
    private final StatusSignal<AngularVelocity> pitchVelocity;
    private final StatusSignal<AngularVelocity> rollVelocity;
    private final StatusSignal<Boolean> faultHardware;

    // StatusSignal buffers for high-freq odometry
    private final DoubleCircularBuffer timestampBuffer;
    private final DoubleCircularBuffer yawSignalBuffer;

    public GyroIOPigeon2(
            final HardwareConstants.GyroConstants gyroConstants,
            final OdometryThreadRunner odometryThreadRunner
    ) {
        this.pigeon = new Pigeon2(gyroConstants.gyroId(), gyroConstants.CANBus());

        this.yaw = pigeon.getYaw(false);
        this.pitch = pigeon.getPitch(false);
        this.roll = pigeon.getRoll(false);
        this.yawVelocity = pigeon.getAngularVelocityZWorld(false);
        this.pitchVelocity = pigeon.getAngularVelocityYWorld(false);
        this.rollVelocity = pigeon.getAngularVelocityXWorld(false);
        this.faultHardware = pigeon.getFault_Hardware(false);

        this.timestampBuffer = odometryThreadRunner.makeTimestampBuffer();
        this.yawSignalBuffer = odometryThreadRunner.registerSignal(pigeon, this.yaw);

        RefreshAll.add(
                RefreshAll.CANBus.CANIVORE,
                yaw,
                pitch,
                roll,
                yawVelocity,
                pitchVelocity,
                rollVelocity,
                faultHardware
        );
    }

    @SuppressWarnings("DuplicatedCode")
    @Override
    public void updateInputs(final GyroIOInputs inputs) {
        inputs.yawPositionDeg = getYaw();
        inputs.pitchPositionDeg = getPitch();
        inputs.rollPositionDeg = getRoll();
        inputs.yawVelocityDegPerSec = this.yawVelocity.getValueAsDouble();
        inputs.pitchVelocityDegPerSec = this.pitchVelocity.getValueAsDouble();
        inputs.rollVelocityDegPerSec = this.rollVelocity.getValueAsDouble();
        inputs.hasHardwareFault = this.faultHardware.getValue();

        inputs.odometryTimestampsSec = OdometryThreadRunner.writeBufferToArray(timestampBuffer);
        timestampBuffer.clear();

        inputs.odometryYawPositionsDeg = OdometryThreadRunner.writeBufferToArray(yawSignalBuffer);
        yawSignalBuffer.clear();
    }

    // TODO: duplicated code warnings here aren't exactly amazing, but I can't really think of a way to extract
    //  them correctly while still keeping with how AdvKit works
    @SuppressWarnings("DuplicatedCode")
    @Override
    public void config() {
        final Pigeon2Configuration pigeon2Configuration = new Pigeon2Configuration();
        pigeon2Configuration.MountPose.MountPoseRoll = -0.2720952332019806;
        pigeon2Configuration.MountPose.MountPosePitch = 0.06304807960987091;
        pigeon2Configuration.MountPose.MountPoseYaw = -1.072310447692871;
        pigeon.getConfigurator().apply(pigeon2Configuration);

        BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                this.pitch,
                this.pitchVelocity,
                this.roll,
                this.rollVelocity
        );
        BaseStatusSignal.setUpdateFrequencyForAll(
                4,
                this.faultHardware
        );
        ParentDevice.optimizeBusUtilizationForAll(4, pigeon);
    }

    public double getYaw() {
        return Phoenix6Utils.latencyCompensateIfSignalIsGood(this.yaw, this.yawVelocity);
    }

    public double getPitch() {
        return Phoenix6Utils.latencyCompensateIfSignalIsGood(this.pitch, this.pitchVelocity);
    }

    public double getRoll() {
        return Phoenix6Utils.latencyCompensateIfSignalIsGood(this.roll, this.rollVelocity);
    }

    @Override
    public void setAngle(final Rotation2d angle) {
        pigeon.setYaw(angle.getDegrees());
    }
}
