package org.firstinspires.ftc.blackice.util;

public final class Utils {
    private Utils() {}
    
    /**
     * Returns the given value if it isn't null, otherwise returns the given default value.
     */
    public static <T> T getOrDefault(T value, T defaultValue) {
        return (value != null) ? value : defaultValue;
    }
    
    // same as getOrDefault(T value, T defaultValue) but converts a Double into a double
    public static double getOrDefault(Double value, double defaultValue) {
        return (value != null) ? value : defaultValue;
    }
}
