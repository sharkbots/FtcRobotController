package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.tools.Buttons;
import org.firstinspires.ftc.teamcode.tools.Global;
import org.firstinspires.ftc.teamcode.tools.StateMachine.StateMachine;
import org.firstinspires.ftc.teamcode.tools.StateMachine.Action;

import java.lang.reflect.Field;
import java.lang.reflect.Modifier;

public class ConfigMenu {

    Buttons buttons;
    Object object;
    Field[] fields;
    int currentField = 0;

    StateMachine sm;
    StateMachine.State navigateMenu, editMenuItem;


    public ConfigMenu(Object object, Buttons buttons) {
        this.buttons = buttons;
        this.object = object;
        this.fields = object.getClass().getDeclaredFields();

        sm = new StateMachine();
        navigateMenu = new StateMachine.State("navigateMenu");
        editMenuItem = new StateMachine.State("editMenuItem");
        sm.setInitialState(navigateMenu);

        navigateMenu.addTransitionTo(editMenuItem, buttons.handlerA::Pressed, new Action("enterEdit", ()->true));
        editMenuItem.addTransitionTo(navigateMenu, buttons.handlerA::Pressed, new Action("nextMenuItem", ()->true));

        navigateMenu.addTransitionTo(navigateMenu, buttons.handlerDPad_Down::Pressed, new Action("nextMenuItem", this::nextMenuItem));
        navigateMenu.addTransitionTo(navigateMenu, buttons.handlerDPad_Up::Pressed, new Action("previousMenuItem", this::previousMenuItem));

        editMenuItem.addTransitionTo(editMenuItem, buttons.handlerDPad_Right::Pressed, new Action("incrementField", this::incrementField));
        editMenuItem.addTransitionTo(editMenuItem, buttons.handlerDPad_Left::Pressed, new Action("decrementField", this::decrementField));
    }

    public void update() {
        buttons.update();
        sm.updateState();
        updateDisplay();
    }

    private boolean incrementField()  {
        Field field = fields[currentField];
        field.setAccessible(true);
        Class<?> type = field.getType();
        Global.telemetry.addLine(type.getName());
        try {
            if (type.equals(Integer.class) || type.equals(int.class)) {
                field.setInt(object, field.getInt(object) + 1);
            } else if (type.equals(Double.class) || type.equals(double.class)) {
                field.setDouble(object, field.getDouble(object) + 0.1);
            } else if (type.equals(Float.class) || type.equals(float.class)) {
                field.setFloat(object, field.getFloat(object) + 0.1f);
            } else if (type.equals(Boolean.class) || type.equals(boolean.class)) {
                Boolean currentValue = (Boolean) field.get(object);
                field.set(object, !currentValue); // Toggle the boolean value
            } else if (type.isEnum()) {
                Object[] enumValues = type.getEnumConstants();
                Object currentValue = field.get(object);
                for (int i = 0; i < enumValues.length; i++) {
                    if (enumValues[i].equals(currentValue)) {
                        Object nextValue = enumValues[(i + 1) % enumValues.length];
                        field.set(object, nextValue);
                        break;
                    }
                }
            }
        }
    catch (IllegalAccessException e) {
        e.printStackTrace();
    }

        return true;
    }

    private boolean decrementField() {
        Field field = fields[currentField];
        field.setAccessible(true);
        Class<?> type = field.getType();
        try {
            if (type.equals(Integer.class) || type.equals(int.class)) {
                field.setInt(object, field.getInt(object) - 1);
            } else if (type.equals(Double.class) || type.equals(double.class)) {
                field.setDouble(object, field.getDouble(object) - 0.1);
            } else if (type.equals(Float.class) || type.equals(float.class)) {
                field.setFloat(object, field.getFloat(object) - 0.1f);
            } else if (type.equals(Boolean.class) || type.equals(boolean.class)) {
                Boolean currentValue = (Boolean) field.get(object);
                field.set(object, !currentValue); // Toggle the boolean value
            } else if (type.isEnum()) {
                Object[] enumValues = type.getEnumConstants();
                Object currentValue = field.get(object);
                for (int i = 0; i < enumValues.length; i++) {
                    if (enumValues[i].equals(currentValue)) {
                        Object nextValue = enumValues[i==0? enumValues.length-1 : (i - 1) % enumValues.length];
                        field.set(object, nextValue);
                        break;
                    }
                }
            }
        }
        catch (IllegalAccessException e) {
            e.printStackTrace();
        }

        return true;
    }

    private boolean previousMenuItem() {
        currentField = Range.clip(currentField-1, 0, fields.length-1);
        return true;
    }

    private boolean nextMenuItem() {
        currentField = Range.clip(currentField+1, 0, fields.length-1);
        return true;
    }

    public void updateDisplay() {
        // Assuming Global.telemetry is accessible and correct.
        try {
            // Display static fields
            for (int i = 0; i < fields.length; i++) {
                Field field = fields[i];
                field.setAccessible(true);
                Object value = field.get(Modifier.isStatic(field.getModifiers())?null:object); // Use null for static fields
                Global.telemetry.addData(formattedFieldName(i), value != null ? formattedValue(i, value.toString()) : "null");
            }

        } catch (IllegalAccessException e) {
            e.printStackTrace();
        }
        Global.telemetry.update();
    }

    private String formattedFieldName(int fieldIndex) {
        return formattedValue(fieldIndex, fields[fieldIndex].getName());
    }

    private String formattedValue(int fieldIndex, String value) {
        if (fieldIndex==currentField) {
            value = bold(value);
            value = color(value, (sm.currentState==navigateMenu)?"cyan":"yellow");
        }
        return value;
    }

    private String bold(String string) {
        return "<b>"+string+"</b>";
    }
    private String italic(String string) {
        return "<i>"+string+"</i>";
    }
    private String color(String string, String color) {
        return "<font color = \""+color+"\">"+string+"</font>";
    }
}