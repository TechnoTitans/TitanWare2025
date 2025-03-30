package frc.robot.utils;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import java.util.function.Supplier;

public class Container<T> implements Supplier<T> {
    public T value;

    @SuppressWarnings("unused")
    public Container() {}

    public Container(T initialValue) {
        value = initialValue;
    }

    @Override
    public T get() {
        return value;
    }

    public Command set(final Supplier<T> valueSupplier) {
        return Commands.runOnce(() -> this.value = valueSupplier.get());
    }

    public Command set(final T value) {
        return set(() -> value);
    }

    public static <T> Container<T> of(final T value) {
        return new Container<>(value);
    }

    public static <T> Container<T> empty() {
        return new Container<>();
    }
}