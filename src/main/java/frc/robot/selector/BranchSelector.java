package frc.robot.selector;

import edu.wpi.first.networktables.*;
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

    private final StringArrayPublisher branchPublisher;
    private final StringSubscriber selectedBranchSubscriber;
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
        this.defaultBranch = BRANCH_TO_SCORE_POSITION.get("LEFT_L2");
        this.selectedBranch = defaultBranch.toString();

        final NetworkTable ntTable = NetworkTableInstance.getDefault().getTable(ntTableName);
        this.branchPublisher = ntTable.getStringArrayTopic(Constants.NetworkTables.SELECTED_BRANCH).publish();
        this.selectedBranchSubscriber = ntTable.getStringTopic(Constants.NetworkTables.DASH_SELECTED_BRANCH)
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
        selectedBranchSubscriber.close();
        branchPublisher.close();
    }

    @Override
    public void periodic() {
        if (!Logger.hasReplaySource()) {
            selectedBranch = selectedBranchSubscriber.get();
        }
        Logger.processInputs(prefix, inputs);
    }
}
