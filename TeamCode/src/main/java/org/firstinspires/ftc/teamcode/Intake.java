package org.firstinspires.ftc.teamcode;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.StateMachine;
import org.firstinspires.ftc.teamcode.tools.Button;
import org.firstinspires.ftc.teamcode.tools.OverrideMotor;
import org.firstinspires.ftc.teamcode.tools.Robot;

public class Intake {
    public final OverrideMotor intakeMotor;
    public final Servo intakeFlipper;
    private final Button handlerLeftTrigger, handlerLeftStick_Up, handlerLeftStick_Down;
    private final double intakeFlipperUp, intakeFlipperPixel5, intakeFlipperPixel4, intakeFlipperPixel3;
    private FLIPPER_CASES flipperCase;

    public Intake(HardwareMap hardwareMap, Button handlerLeftTrigger, Button handlerLeftStick_Up, Button handlerLeftStick_Down) {
        // Motors
        intakeMotor = new OverrideMotor(hardwareMap.dcMotor.get("intakeMotor"));
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeFlipper = hardwareMap.servo.get("intakeFlipper");

        intakeFlipperUp = 1.0;
        intakeFlipperPixel5 = 0.62;
        intakeFlipperPixel4 = 0.6075;
        intakeFlipperPixel3 = 0.595;

        intakeFlipper.scaleRange(0, 1.0);

        this.handlerLeftTrigger = handlerLeftTrigger;
        this.handlerLeftStick_Up = handlerLeftStick_Up;
        this.handlerLeftStick_Down = handlerLeftStick_Down;

        flipperCase = FLIPPER_CASES.UP;
    }

    private enum FLIPPER_CASES{
        UP, PIXEL_5, PIXEL_4, PIXEL_3
    }

    public void update(Boolean notInOutTake){
        // Manages Reject mode on Roomba as an override of its current power and state
        if(handlerLeftTrigger.Pressed()) {
            intakeMotor.setOverridePower(-1);
        } else if (handlerLeftTrigger.Released()) {
            intakeMotor.cancelOverridePower();
        }
        if(notInOutTake){
            switch (flipperCase){
                case UP:
                    if(handlerLeftStick_Down.Pressed()){
                        intakeFlipper.setPosition(intakeFlipperPixel5);
                        flipperCase = FLIPPER_CASES.PIXEL_5;
                    }
                    break;

                case PIXEL_5:
                    if(handlerLeftStick_Down.Pressed()){
                        intakeFlipper.setPosition(intakeFlipperPixel4);
                        flipperCase = FLIPPER_CASES.PIXEL_4;
                    }
                    if(handlerLeftStick_Up.Pressed()){
                        intakeFlipper.setPosition(intakeFlipperUp);
                        flipperCase = FLIPPER_CASES.UP;
                    }
                    break;

                case PIXEL_4:
                    if(handlerLeftStick_Down.Pressed()){
                        intakeFlipper.setPosition(intakeFlipperPixel3);
                        flipperCase = FLIPPER_CASES.PIXEL_3;
                    }
                    if(handlerLeftStick_Up.Pressed()){
                        intakeFlipper.setPosition(intakeFlipperPixel5);
                        flipperCase = FLIPPER_CASES.PIXEL_5;
                    }
                    break;

                case PIXEL_3:
                    if(handlerLeftStick_Up.Pressed()){
                        intakeFlipper.setPosition(intakeFlipperPixel4);
                        flipperCase = FLIPPER_CASES.PIXEL_4;
                    }
                    break;
            }
        }
    }

    public void reset(){
        intakeFlipper.setPosition(1.0);
    }

}
