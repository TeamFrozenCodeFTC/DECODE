package org.firstinspires.ftc.blackice.util;

import android.util.Log;

import androidx.annotation.Nullable;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.lang.reflect.Field;


public final class Logger {
    private Logger() {}
    
    private static final int LEVEL = Log.VERBOSE;
    private static final boolean USE_FTC_DASHBOARD = true;
    private static @Nullable Telemetry telemetry;
    
    public static void provideTelemetry(Telemetry driverStationTelemetry) {
        if (USE_FTC_DASHBOARD) {
            telemetry = new MultipleTelemetry(driverStationTelemetry,
                                              FtcDashboard.getInstance().getTelemetry());
        }
        else {
            telemetry = driverStationTelemetry;
        }
    }
    
    public static void initializeGraphKeys(String... keys) {
        if (!USE_FTC_DASHBOARD) return;
        
        for (String key : keys) {
            graph(key, 0);
        }
        updateGraph();
    }
    
    public static void updateGraph() {
        if (!USE_FTC_DASHBOARD) return;
        
        if (telemetry != null) {
            telemetry.update();
        }
        else {
            FtcDashboard.getInstance().getTelemetry().update();
        }
    }
    
    public static void graph(String key, double value) {
        if (!USE_FTC_DASHBOARD) return;
        
        if (telemetry != null) {
            telemetry.addData(key, value);
        }
        else {
            FtcDashboard.getInstance().getTelemetry().addData(key, value);
        }
    }
    
    private static void graphIfNumber(String key, Object value) {
        if (value instanceof Number) {
            graph(key, ((Number) value).doubleValue());
        }
    }
    
    public static void debug(String tag, String key, Object value) {
        if (LEVEL > Log.DEBUG) return;
        graphIfNumber(key, value);
        Log.d("BlackIce" + tag, key + ": " + value);
    }
    
    /**
     * Log information in verbose level (aka that breaks down everything detailed e.g.
     * logging every single loop).
     */
    public static void verbose(String key, Object value) {
        if (LEVEL > Log.VERBOSE) return;
        graphIfNumber(key, value);
        Log.v("BlackIce", key + ": " + value);
    }
    
    public static void debug(String key, Object value) {
        debug("", key, value);
    }
    
    public static void debug(String message) {
        debug("", "", message);
    }
    
    public static void divide(String message) {
        debug("", "", message + " ------------------------");
    }
    
    public static void error(String message) {
        Log.e("BlackIce", message);
    }
    
    public static void warn(String message) {
        Log.w("BlackIce", message);
    }
    
    public static void warnWithStack(String message) {
        Log.w("BlackIce", message + Log.getStackTraceString(new Throwable()));
    }
    
    public static void info(String key, Object value) {
        info(key + ": " + value.toString());
    }
    
    public static void info(String message) {
        if (LEVEL > Log.INFO) return;
        Log.i("BlackIce", message);
    }
    
    public static void logFields(String tag, Object obj) {
        if (obj == null) {
            debug(tag, "null object");
            return;
        }
        
        Class<?> clazz = obj.getClass();
        for (Field field : clazz.getDeclaredFields()) {
            field.setAccessible(true);
            try {
                Object value = field.get(obj);
                debug(tag, field.getName() + " = " + value);
            } catch (IllegalAccessException e) {
                debug(tag, "Cannot access field: " + field.getName());
            }
        }
    }
}
