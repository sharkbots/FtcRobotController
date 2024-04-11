package org.firstinspires.ftc.teamcode.tools;

import com.qualcomm.robotcore.hardware.Gamepad;

public class Buttons {
    public final Button handlerA;
    public final Button handlerB;
    public final Button handlerX;
    public final Button handlerY;
    public final Button handlerLeftBumper;
    public final Button handlerRightBumper;
    public final Button handlerLeftTrigger;
    public final Button handlerRightTrigger;
    public final Button handlerDPad_Down;
    public final Button handlerDPad_Up;
    public final Button handlerDPad_Left;
    public final Button handlerDPad_Right;
    public final Button handlerLeftStick_Up;
    public final Button handlerLeftStick_Down;
    public final Button handlerLeftStick_Left;
    public final Button handlerLeftStick_Right;

    // public for now - ultimately it needs to be fully encapsulated by adding float value for each button
    // and return either the raw or the enhanced one for sticks and boolean ones
    public final Gamepad gamepad1;
    public final Gamepad gamepad2;

    public Buttons(Gamepad gamepad1, Gamepad gamepad2) {
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;

        // Button assignment
        handlerA = new Button(this.gamepad2, Button.NAME.A);
        handlerB = new Button(this.gamepad2, Button.NAME.B);
        handlerX = new Button(this.gamepad2, Button.NAME.X);
        handlerY = new Button(this.gamepad2, Button.NAME.Y);
        handlerLeftBumper = new Button(this.gamepad2, Button.NAME.LEFT_BUMPER);
        handlerRightBumper = new Button(this.gamepad2, Button.NAME.RIGHT_BUMPER);
        handlerLeftTrigger = new Button(this.gamepad2, Button.NAME.LEFT_TRIGGER);
        handlerRightTrigger = new Button(this.gamepad2, Button.NAME.RIGHT_TRIGGER);
        handlerDPad_Down = new Button(this.gamepad2, Button.NAME.DPAD_DOWN);
        handlerDPad_Up = new Button(this.gamepad2, Button.NAME.DPAD_UP);
        handlerDPad_Left = new Button(this.gamepad2, Button.NAME.DPAD_LEFT);
        handlerDPad_Right = new Button(this.gamepad2, Button.NAME.DPAD_RIGHT);
        handlerLeftStick_Up = new Button(this.gamepad2, Button.NAME.LEFT_STICK_UP);
        handlerLeftStick_Down = new Button(this.gamepad2, Button.NAME.LEFT_STICK_DOWN);
        handlerLeftStick_Left = new Button(this.gamepad2, Button.NAME.LEFT_STICK_LEFT);
        handlerLeftStick_Right = new Button(this.gamepad2, Button.NAME.LEFT_STICK_RIGHT);

    }
    public void update() {
        handlerA.updateButton(gamepad2);
        handlerB.updateButton(gamepad2);
        handlerX.updateButton(gamepad2);
        handlerY.updateButton(gamepad2);
        handlerLeftBumper.updateButton(gamepad2);
        handlerRightBumper.updateButton(gamepad2);
        handlerLeftTrigger.updateButton(gamepad2);
        handlerRightTrigger.updateButton(gamepad2);
        handlerDPad_Down.updateButton(gamepad2);
        handlerDPad_Up.updateButton(gamepad2);
        handlerDPad_Left.updateButton(gamepad2);
        handlerDPad_Right.updateButton(gamepad2);
        handlerLeftStick_Up.updateButton(gamepad2);
        handlerLeftStick_Down.updateButton(gamepad2);
        handlerLeftStick_Left.updateButton(gamepad2);
        handlerLeftStick_Right.updateButton(gamepad2);
    }
}
