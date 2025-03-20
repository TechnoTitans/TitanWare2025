package frc.robot.selector;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringSubscriber;
import frc.robot.ScoreCommands;
import frc.robot.constants.Constants;
import frc.robot.constants.FieldConstants;
import frc.robot.utils.closeables.ToClose;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.littletonrobotics.junction.networktables.LoggedNetworkInput;

import java.util.Map;

public class BranchSelector extends LoggedNetworkInput implements AutoCloseable {
    private static final Map<String, ScoreCommands.ScorePosition> BRANCH_TO_SCORE_POSITION = Map.of(
            "LEFT_L4", new ScoreCommands.ScorePosition(FieldConstants.Reef.Side.LEFT, ScoreCommands.Level.L4),
            "RIGHT_L4", new ScoreCommands.ScorePosition(FieldConstants.Reef.Side.RIGHT, ScoreCommands.Level.L4),
            "LEFT_L3", new ScoreCommands.ScorePosition(FieldConstants.Reef.Side.LEFT, ScoreCommands.Level.L3),
            "RIGHT_L3", new ScoreCommands.ScorePosition(FieldConstants.Reef.Side.RIGHT, ScoreCommands.Level.L3),
            "LEFT_L2", new ScoreCommands.ScorePosition(FieldConstants.Reef.Side.LEFT, ScoreCommands.Level.L2),
            "RIGHT_L2", new ScoreCommands.ScorePosition(FieldConstants.Reef.Side.RIGHT, ScoreCommands.Level.L2),
            "LEFT_L1", new ScoreCommands.ScorePosition(FieldConstants.Reef.Side.LEFT, ScoreCommands.Level.L1),
            "RIGHT_L1", new ScoreCommands.ScorePosition(FieldConstants.Reef.Side.RIGHT, ScoreCommands.Level.L1)
    );

    private final String ntTableName;

    private final StringPublisher robotBranchPublisher;
    private final StringSubscriber dashSelectedBranchSubscriber;
    private final LoggableInputs inputs = new LoggableInputs() {
        @Override
        public void toLog(LogTable table) {
            table.put(ntTableName, selectedBranch);
        }

        @Override
        public void fromLog(LogTable table) {
            selectedBranch = table.get(ntTableName, selectedBranch);
        }
    };

    private final ScoreCommands.ScorePosition defaultBranch;
    private String selectedBranch;

    public BranchSelector() {
        this.ntTableName = Constants.NetworkTables.BRANCH_TABLE;
        this.selectedBranch = "LEFT_L2";
        this.defaultBranch = BRANCH_TO_SCORE_POSITION.get(selectedBranch);

        final NetworkTable ntTable = NetworkTableInstance.getDefault().getTable(ntTableName);
        this.robotBranchPublisher = ntTable.getStringTopic(Constants.NetworkTables.SELECTED_BRANCH).publish();
        robotBranchPublisher.set(selectedBranch);

        final StringPublisher clearPreviousSelection =
                ntTable.getStringTopic(Constants.NetworkTables.DASH_SELECTED_BRANCH).publish();
        clearPreviousSelection.set(selectedBranch);
        clearPreviousSelection.close();

        this.dashSelectedBranchSubscriber = ntTable.getStringTopic(Constants.NetworkTables.DASH_SELECTED_BRANCH)
                .subscribe(selectedBranch);

        Logger.registerDashboardInput(this);
        ToClose.add(this);
    }

    public ScoreCommands.ScorePosition getSelected() {
        final ScoreCommands.ScorePosition selected = BRANCH_TO_SCORE_POSITION.get(selectedBranch);
        return selected != null ? selected : defaultBranch;
    }

    @Override
    public void close() {
        dashSelectedBranchSubscriber.close();
        robotBranchPublisher.close();
    }

    @Override
    public void periodic() {
        if (!Logger.hasReplaySource()) {
            selectedBranch = dashSelectedBranchSubscriber.get();
            robotBranchPublisher.set(selectedBranch);
        }
        Logger.processInputs(prefix, inputs);
    }
}
