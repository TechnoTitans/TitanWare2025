package frc.robot.utils.sim.motors;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.constants.SimConstants;
import frc.robot.utils.sim.feedback.SimFeedbackSensor;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class TalonFXSim implements SimMotorController {
    private final List<TalonFXSimState> simStates;
    private final Consumer<Double> update;
    private final Consumer<Double> inputVoltage;
    private final Supplier<Double> angularPositionRads;
    private final Supplier<Double> angularVelocityRadsPerSec;
    private final double gearRatio;

    private final TalonFXSimState useSimStateVoltage;
    private final TalonFXSimState useSimStateTorqueCurrent;

    private boolean hasRemoteSensor = false;
    private SimFeedbackSensor feedbackSensor;

    private TalonFXSim(
            final List<TalonFX> talonFXControllers,
            final List<TalonFXSimState> simStates,
            final double gearRatio,
            final Consumer<Double> update,
            final Consumer<Double> inputVoltage,
            final Supplier<Double> angularPositionRads,
            final Supplier<Double> angularVelocityRadsPerSec
    ) {
        if (talonFXControllers.isEmpty() || simStates.isEmpty()) {
            throw new IllegalArgumentException("TalonFX must not be empty! TalonFXSimStates must not be empty!");
        }

        this.simStates = simStates;
        this.update = update;
        this.inputVoltage = inputVoltage;
        this.angularPositionRads = angularPositionRads;
        this.angularVelocityRadsPerSec = angularVelocityRadsPerSec;
        this.gearRatio = gearRatio;

        this.useSimStateVoltage = simStates.get(0);
        this.useSimStateTorqueCurrent = simStates.get(0);
    }

    public TalonFXSim(
            final List<TalonFX> talonFXControllers,
            final double gearRatio,
            final Consumer<Double> update,
            final Consumer<Double> inputVoltage,
            final Supplier<Double> angularPositionRads,
            final Supplier<Double> angularVelocityRadsPerSec
    ) {
        this(
                talonFXControllers,
                getSimStatesFromControllers(talonFXControllers),
                gearRatio,
                update,
                inputVoltage,
                angularPositionRads,
                angularVelocityRadsPerSec
        );
    }

    public TalonFXSim(
            final TalonFX talonFX,
            final double gearRatio,
            final Consumer<Double> update,
            final Consumer<Double> inputVoltage,
            final Supplier<Double> angularPositionRads,
            final Supplier<Double> angularVelocityRadsPerSec
    ) {
        this(
                List.of(talonFX),
                gearRatio,
                update,
                inputVoltage,
                angularPositionRads,
                angularVelocityRadsPerSec
        );
    }

    private static List<TalonFXSimState> getSimStatesFromControllers(final List<TalonFX> talonFXControllers) {
        final ArrayList<TalonFXSimState> simStates = new ArrayList<>();
        for (final TalonFX talonFX : talonFXControllers) {
            simStates.add(talonFX.getSimState());
        }
        return simStates;
    }

    @Override
    public void attachFeedbackSensor(final SimFeedbackSensor feedbackSensor) {
        if (hasRemoteSensor) {
            throw new RuntimeException("Attempt to attach SimFeedbackSensor when one is already attached!");
        }

        this.hasRemoteSensor = true;
        this.feedbackSensor = feedbackSensor;
    }

    @Override
    public void update(final double dt) {
        final double motorVoltage = getMotorVoltage();
        inputVoltage.accept(motorVoltage);
        update.accept(dt);

        final double mechanismAngularPositionRots = getAngularPositionRots();
        final double mechanismAngularVelocityRotsPerSec = getAngularVelocityRotsPerSec();

        for (final TalonFXSimState simState : simStates) {
            simState.setRawRotorPosition(gearRatio * mechanismAngularPositionRots);
            simState.setRotorVelocity(gearRatio * mechanismAngularVelocityRotsPerSec);
            simState.setSupplyVoltage(
                    RobotController.getBatteryVoltage() -
                            (simState.getSupplyCurrent() * SimConstants.FALCON_MOTOR_RESISTANCE)
            );
        }

        if (hasRemoteSensor) {
            feedbackSensor.setRawPosition(mechanismAngularPositionRots);
            feedbackSensor.setVelocity(mechanismAngularVelocityRotsPerSec);
        }
    }

    @Override
    public void rawUpdate(final double mechanismPositionRots, final double mechanismVelocityRotsPerSec) {
        for (final TalonFXSimState simState : simStates) {
            simState.setRawRotorPosition(gearRatio * mechanismPositionRots);
            simState.setRotorVelocity(gearRatio * mechanismVelocityRotsPerSec);
        }

        if (hasRemoteSensor) {
            feedbackSensor.setRawPosition(mechanismPositionRots);
            feedbackSensor.setVelocity(mechanismVelocityRotsPerSec);
        }
    }

    @Override
    public double getAngularPositionRots() {
        return Units.radiansToRotations(angularPositionRads.get());
    }

    @Override
    public double getAngularVelocityRotsPerSec() {
        return Units.radiansToRotations(angularVelocityRadsPerSec.get());
    }

    @Override
    public double getMotorVoltage() {
        return useSimStateVoltage.getMotorVoltage();
    }

    @Override
    public double getMotorCurrent() {
        return useSimStateTorqueCurrent.getTorqueCurrent();
    }
}
