package frc.robot.state;

import frc.robot.constants.FieldConstants.Reef;

import java.util.HashMap;
import java.util.Map;
import java.util.Optional;

public class ReefState {
    public enum BranchState {
        NONE,
        CORAL
    }

    public record Branch (Reef.Face face, Reef.Side side, Reef.Level level) {}

    private final Map<Reef.Face, Map<Reef.Side, Map<Reef.Level, BranchState>>> reef = new HashMap<>();

    public ReefState() {
        for (final Reef.Face face : Reef.Face.values()) {
            reef.put(face, new HashMap<>());
            for (final Reef.Side side : Reef.Side.values()) {
                reef.get(face).put(side, new HashMap<>());
                for (final Reef.Level level : Reef.Level.values()) {
                    reef.get(face).get(side).put(level, BranchState.NONE);
                }
            }
        }
    }

    public void setBranchState(final Branch branch, final BranchState state) {
        reef.get(branch.face()).get(branch.side()).put(branch.level(), state);
    }

    public BranchState getBranchState(final Reef.Face face, final Reef.Side side, final Reef.Level level) {
        return reef.get(face).get(side).get(level);
    }

    public Optional<Branch> getNextBranch(final Reef.Face face) {
        for (final Reef.Level level : Reef.Level.values()) {
            for (final Reef.Side side : Reef.Side.values()) {
                if (getBranchState(face, side, level) == BranchState.NONE) {
                    return Optional.of(new Branch(face, side, level));
                }
            }
        }
        return Optional.empty();
    }

    public Optional<Branch> getAndSetNextBranch(final Reef.Face face) {
        final Optional<Branch> maybeBranch = getNextBranch(face);
        maybeBranch.ifPresent(branch -> setBranchState(branch, BranchState.CORAL));
        return maybeBranch;
    }

    public void reset() {
        for (final Reef.Face face : Reef.Face.values()) {
            for (final Reef.Side side : Reef.Side.values()) {
                for (final Reef.Level level : Reef.Level.values()) {
                    reef.get(face).get(side).put(level, BranchState.NONE);
                }
            }
        }
    }
}
