package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.tools.Buttons;
import org.firstinspires.ftc.teamcode.tools.Global;
import org.firstinspires.ftc.teamcode.tools.stateMachine.StateMachine;
import org.firstinspires.ftc.teamcode.tools.stateMachine.Action;

import java.lang.reflect.Field;
import java.lang.reflect.Modifier;

public class ConfigMenu {

    Buttons buttons;
    Object object;
    Object fieldBackup = null;

    Field[] fields;
    int currentField = 0;

    StateMachine sm;
    StateMachine.State navigateMenu, editMenuItem, lockMenu;


    public ConfigMenu(Object object, Buttons buttons) {
        this.buttons = buttons;
        this.object = object;
        this.fields = object.getClass().getDeclaredFields();

        sm = new StateMachine();
        navigateMenu = new StateMachine.State("navigateMenu");
        editMenuItem = new StateMachine.State("editMenuItem");
        lockMenu = new StateMachine.State("lockMenu");
        sm.setInitialState(navigateMenu);

        navigateMenu.addTransitionTo(editMenuItem, buttons.handlerA::Pressed, new Action("enterEdit", this::backupCurrentField));
        editMenuItem.addTransitionTo(navigateMenu, buttons.handlerA::Pressed, new Action("nextMenuItem", ()->{fieldBackup=null; return true;}));
        editMenuItem.addTransitionTo(navigateMenu, buttons.handlerB::Pressed, new Action("nextMenuItem", this::restoreCurrentField));

        navigateMenu.addTransitionTo(navigateMenu, buttons.handlerDPad_Down::Pressed, new Action("nextMenuItem", this::nextMenuItem));
        navigateMenu.addTransitionTo(navigateMenu, buttons.handlerDPad_Up::Pressed, new Action("previousMenuItem", this::previousMenuItem));

        editMenuItem.addTransitionTo(editMenuItem, buttons.handlerDPad_Right::Pressed, new Action("incrementField", ()->changeFieldBy(1.0)));
        editMenuItem.addTransitionTo(editMenuItem, buttons.handlerDPad_Left::Pressed, new Action("decrementField",  ()->changeFieldBy(-1.0)));

        editMenuItem.addTransitionTo(editMenuItem, buttons.handlerRightBumper::Pressed, new Action("incrementField", ()->changeFieldBy(0.1)));
        editMenuItem.addTransitionTo(editMenuItem, buttons.handlerLeftBumper::Pressed, new Action("decrementField",  ()->changeFieldBy(-0.1)));

        lockMenu.addTransitionTo(lockMenu, buttons.handlerX::Pressed, new Action("menu locked", this::lockMenu));
    }


    private Boolean backupCurrentField() {
        Field field = fields[currentField];
        field.setAccessible(true);
        try {
            fieldBackup = field.get(object);
        } catch (IllegalAccessException e) {
            e.printStackTrace();
        }
        return true;
    }


    private Boolean restoreCurrentField() {
        Field field = fields[currentField];
        field.setAccessible(true);
        try {
             field.set(object, fieldBackup);
        } catch (IllegalAccessException e) {
            e.printStackTrace();
        }
        return true;
    }



    public void update() {
        buttons.update();
        sm.updateState();
        updateDisplay();
    }

    private boolean changeFieldBy(double value)  {
        assert(value!=0);
        Field field = fields[currentField];
        field.setAccessible(true);
        Class<?> type = field.getType();
        Global.telemetry.addLine(type.getName());
        int intValue = (int)Math.round(value); // used of integer like values below after unboxing field
        if(intValue==0) { // case where double input is within ]-0.5;+0.5[
            intValue = value>=0? 1:-1;
        }

        try {
            if (type.equals(Integer.class) || type.equals(int.class)) {
                field.setInt(object, field.getInt(object) + intValue);
            } else if (type.equals(Double.class) || type.equals(double.class)) {
                field.setDouble(object, field.getDouble(object) + value);
            } else if (type.equals(Float.class) || type.equals(float.class)) {
                field.setFloat(object, field.getFloat(object) + (float)value);
            } else if (type.equals(Boolean.class) || type.equals(boolean.class)) {
                Boolean currentValue = (Boolean) field.get(object);
                field.set(object, !currentValue); // Toggle the boolean value
            } else if (type.isEnum()) {
                Object[] enumValues = type.getEnumConstants();
                Object currentValue = field.get(object);
                for (int i = 0; i < enumValues.length; i++) {
                    if (enumValues[i].equals(currentValue)) {
                        int len = enumValues.length;
                        Object nextValue = enumValues[(((i + intValue) % len) + len) % len];//Java modulo conserves sign: -1%5 = -1 and not 4, need positive index all the time
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

    private boolean lockMenu(){

        return true;
    }
    private boolean nextMenuItem() {
        currentField = Range.clip(currentField+1, 0, fields.length-1);
        return true;
    }

    public void updateDisplay() {
        // Assuming Global.telemetry is accessible and correct.
        try {
            if(sm.currentState!=lockMenu) {
                // Display static fields
                for (int i = 0; i < fields.length; i++) {
                    Field field = fields[i];
                    field.setAccessible(true);
                    Object value = field.get(Modifier.isStatic(field.getModifiers()) ? null : object); // Use null for static fields
                    Global.telemetry.addData(formattedFieldName(i), value != null ? formattedValue(i, value.toString()) : "null");
                }
            }
            else {
                Global.telemetry.addLine("Menu values are locked.");
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
