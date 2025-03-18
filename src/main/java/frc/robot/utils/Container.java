package frc.robot.utils;

public class Container<T> {
    public T value;

    @SuppressWarnings("unused")
    public Container() {}

    public Container(T initialValue) {
        value = initialValue;
    }

    public static <T> Container<T> of(final T value) {
        return new Container<>(value);
    }

    public static <T> Container<T> empty() {
        return new Container<>();
    }
}