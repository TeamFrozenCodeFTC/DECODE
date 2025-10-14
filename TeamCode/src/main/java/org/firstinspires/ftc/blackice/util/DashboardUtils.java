package org.firstinspires.ftc.blackice.util;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.ValueProvider;

import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.util.HashSet;
import java.util.Set;

public class DashboardUtils {
    private static final Set<Object> visited = new HashSet<>();
    
    public static void addPublicFieldsRecursive(Object obj, String path) {
        if (obj == null || visited.contains(obj)) return;
        visited.add(obj);
        
        for (Field field : obj.getClass().getFields()) {
            try {
                Object value = field.get(obj);
                if (value == null) continue;
                
                String fullName = path + "." + field.getName();
                boolean isFinal = Modifier.isFinal(field.getModifiers());
                
                // Register numeric fields if not final
                if (!isFinal) registerNumericField(obj, field, fullName);
                
                // Recurse into reference types
                if (!field.getType().isPrimitive() &&
                    !field.getType().getName().startsWith("java.") &&
                    !field.getType().getName().startsWith("org.firstinspires.ftc.robotcore")) {
                    
                    addPublicFieldsRecursive(value, fullName);
                }
                
            } catch (IllegalAccessException ignored) {}
        }
    }
    
    private static void registerNumericField(Object obj, Field field, String name) {
        if (field.getType() == double.class) registerField(obj, field, name, true);
        else if (field.getType() == int.class) registerField(obj, field, name, false);
    }
    
    private static void registerField(Object obj, Field field, String name, boolean isDouble) {
        FtcDashboard.getInstance().addConfigVariable("Robot", name, new ValueProvider<Object>() {
            public Object get() {
                try {
                    return isDouble ? field.getDouble(obj) : field.getInt(obj);
                } catch (Exception e) { return isDouble ? 0.0 : 0; }
            }
            public void set(Object val) {
                try {
                    if (isDouble) field.setDouble(obj, ((Number) val).doubleValue());
                    else field.setInt(obj, ((Number) val).intValue());
                } catch (Exception ignored) {}
            }
        });
    }
}
