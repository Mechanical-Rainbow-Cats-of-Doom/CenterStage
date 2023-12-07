package org.firstinspires.ftc.teamcode.common.util;

import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Detects any changes in static variables of a class.
 *
 * This is particularly useful when using a Config {@link com.acmerobotics.dashboard.config.Config}
 */
public class ConfigChangeDetector<T> {
    public final Class<T> clazz;
    private final Field[] watching;
    private final Object[] fieldValues;
    private boolean hasChanged;

    public ConfigChangeDetector(Class<T> clazz, List<String> fieldsToIgnore) {
        this.clazz = clazz;
        Field[] fields = clazz.getFields();
        List<Field> staticFields = new ArrayList<>();
        for (Field field : fields) {
            if(!fieldsToIgnore.contains(field.getName()) && Modifier.isStatic(field.getModifiers())) {
                staticFields.add(field);
            }
        }
        this.watching = staticFields.toArray(new Field[0]);
        this.fieldValues = new Object[watching.length];
        for (int i = 0; i < watching.length; i++) {
            try {
                fieldValues[i] = watching[i].get(null);
            } catch (IllegalAccessException e) {
                e.printStackTrace();
            }
        }
    }

    public ConfigChangeDetector(Class<T> clazz) {
        this(clazz, Collections.emptyList());
    }

    public void update() {
        for (int i = 0; i < watching.length; i++) {
            Object newValue = null;
            try {
                newValue = watching[i].get(null);
            } catch (IllegalAccessException e) {
                e.printStackTrace();
            }
            if(!fieldValues[i].equals(newValue)) hasChanged = true;
            fieldValues[i] = newValue;
        }
    }

    public boolean hasChanged() {
        return hasChanged;
    }

    public boolean updateAndHasChanged() {
        update();
        return hasChanged;
    }
}
