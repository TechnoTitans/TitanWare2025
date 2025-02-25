package frc.robot.auto;

import choreo.auto.AutoRoutine;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringArrayPublisher;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.constants.Constants;
import frc.robot.utils.closeables.ToClose;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.littletonrobotics.junction.networktables.LoggedNetworkInput;

import java.util.*;

public class AutoChooser extends LoggedNetworkInput implements AutoCloseable {
    private final String ntTableName;

    private final StringArrayPublisher autoPublisher;
    private final StringSubscriber selectedAutoSubscriber;
    private final LinkedHashMap<String, AutoOption> autoMap;
    private final HashSet<AutoOption> ignoredSet;
    private final LoggableInputs inputs = new LoggableInputs() {
        @Override
        public void toLog(LogTable table) {
            table.put(ntTableName + "/DashboardSelectedAuto", dashboardSelectedAuto);
            table.put(ntTableName + "/SelectedAuto", selectedAuto);
        }

        @Override
        public void fromLog(LogTable table) {
            dashboardSelectedAuto = table.get(ntTableName + "/DashboardSelectedAuto", dashboardSelectedAuto);
            selectedAuto = table.get(ntTableName + "/SelectedAuto", selectedAuto);
        }
    };

    private final AutoOption defaultAuto;
    private String dashboardSelectedAuto;
    private String selectedAuto;
    private AutoRoutine selectedAutoRoutine;
    private DriverStation.Alliance selectedAutoAlliance;

    public AutoChooser(final AutoOption defaultAuto) {
        Objects.requireNonNull(defaultAuto, "Default auto cannot be null");

        this.ntTableName = Constants.NetworkTables.AUTO_TABLE;
        this.ignoredSet = new HashSet<>();
        this.defaultAuto = defaultAuto;
        this.dashboardSelectedAuto = defaultAuto.name();
        this.selectedAuto = dashboardSelectedAuto;

        final NetworkTable ntTable = NetworkTableInstance.getDefault().getTable(ntTableName);
        this.autoPublisher = ntTable.getStringArrayTopic(Constants.NetworkTables.AUTO_PUBLISHER).publish();
        this.selectedAutoSubscriber = ntTable.getStringTopic(Constants.NetworkTables.AUTO_SELECTED_SUBSCRIBER).subscribe(dashboardSelectedAuto);

        this.autoMap = new LinkedHashMap<>(Map.of(selectedAuto, defaultAuto));

        Logger.registerDashboardInput(this);
        ToClose.add(this);
    }

    private boolean shouldIgnoreOption(final AutoOption autoOption) {
        return ignoredSet.contains(autoOption) || !autoOption.hasCompetitionType(Constants.CURRENT_COMPETITION_TYPE);
    }

    public void addOption(final String name, final AutoOption autoOption) {
        if (shouldIgnoreOption(autoOption)) {
            ignoredSet.add(autoOption);
        } else {
            autoMap.put(name, autoOption);
            autoPublisher.set(autoMap.keySet().toArray(String[]::new));
        }
    }

    /**
     * Adds an {@link AutoOption} object to the by using the name of the option
     *
     * @param autoOption the {@link AutoOption}
     * @see AutoOption
     * @see AutoChooser#addOption(String, AutoOption)
     */
    public void addAutoOption(final AutoOption autoOption) {
        addOption(autoOption.name(), autoOption);
    }

    public AutoRoutine getSelected() {
        if (RobotBase.isSimulation()) {
            select(dashboardSelectedAuto, true);
        }
        return selectedAutoRoutine;
    }

    public AutoOption get(final String name) {
        return autoMap.get(name);
    }

    @Override
    public void close() {
        selectedAutoSubscriber.close();
        autoPublisher.close();
    }

    private void select(final String autoName, final boolean force) {
        dashboardSelectedAuto = autoName;

        final Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
        if (dashboardSelectedAuto.equals(selectedAuto) &&
                (alliance.isPresent() && selectedAutoAlliance == alliance.get())) {
            // early return if the selected auto matches the active auto
            return;
        }

        final boolean dsValid = DriverStation.isDisabled() && alliance.isPresent();
        if (dsValid || force) {
            if (!autoMap.containsKey(dashboardSelectedAuto)) {
                return;
            }
            selectedAutoAlliance = alliance.orElse(null);
            selectedAuto = dashboardSelectedAuto;
            selectedAutoRoutine = autoMap.get(selectedAuto).autoRoutine().get();
        } else {
            selectedAutoAlliance = null;
            selectedAuto = defaultAuto.name();
            selectedAutoRoutine = defaultAuto.autoRoutine().get();
        }
    }

    @Override
    public void periodic() {
        if (!Logger.hasReplaySource()) {
            select(selectedAutoSubscriber.get(), false);
        }

        Logger.processInputs(prefix, inputs);
    }
}
