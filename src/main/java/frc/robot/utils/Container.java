package frc.robot.utils;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import java.util.NoSuchElementException;
import java.util.function.Supplier;

public class Container<T> implements Supplier<T> {
    private T value;

    @SuppressWarnings("unused")
    public Container() {}

    public Container(T initialValue) {
        this.value = initialValue;
    }

    @Override
    public T get() {
        if (value == null) {
            throw new NoSuchElementException("No value present");
        }
        return value;
    }

    public void set(final T value) {
        this.value = value;
    }

    public Command setCommand(final Supplier<T> valueSupplier) {
        return Commands.runOnce(() -> this.set(valueSupplier.get())).withName("Container");
    }

    public Command setCommand(final T value) {
        return setCommand(() -> value);
    }

    public static <T> Container<T> of(final T value) {
        return new Container<>(value);
    }

    public static <T> Container<T> empty() {
        return new Container<>();
    }
}