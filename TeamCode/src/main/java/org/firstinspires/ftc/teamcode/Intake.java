package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.tools.Button;
import org.firstinspires.ftc.teamcode.tools.MotorActionManager;
import org.firstinspires.ftc.teamcode.tools.OverrideMotor;
import org.firstinspires.ftc.teamcode.tools.ServoActionManager;

import java.util.HashMap;
import java.util.Map;

public class Intake {
    public final OverrideMotor intakeMotor;
    public final Servo intakeFlipper;
    private final Button handlerLeftTrigger, handlerLeftStick_Up, handlerLeftStick_Down;
    private static final double intakeFlipperUp = 1.0, intakeFlipperPixel5 = 0.62, intakeFlipperPixel4 = 0.6075, intakeFlipperPixel3 = 0.595;
    private flipperPositions flipperCase;
    public MotorActionManager intakeMotorActionManager;
    public ServoActionManager intakeFlipperActionManger;

    public Intake(HardwareMap hardwareMap, Button handlerLeftTrigger, Button handlerLeftStick_Up, Button handlerLeftStick_Down) {
        // Motors
        intakeMotor = new OverrideMotor((DcMotorEx)hardwareMap.dcMotor.get("intakeMotor"));
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeFlipper = hardwareMap.servo.get("intakeFlipper");

        intakeMotorActionManager = new MotorActionManager(intakeMotor);
        intakeFlipperActionManger = new ServoActionManager(intakeFlipper);

        intakeFlipper.scaleRange(0, 1.0);

        this.handlerLeftTrigger = handlerLeftTrigger;
        this.handlerLeftStick_Up = handlerLeftStick_Up;
        this.handlerLeftStick_Down = handlerLeftStick_Down;

        flipperCase = flipperPositions.UP;
    }


    public void startIntakeMotorWithNoEncoder(double power) {
        intakeMotorActionManager.startMotorNoEncoder(power);
    }

    public void stopIntakeMotor() {
        intakeMotorActionManager.stopMotor();
    }

    public enum flipperPositions{
        UP, PIXEL5, PIXEL4, PIXEL3
    }
    private static final Map<Intake.flipperPositions, Double> flipperPositionValues = new HashMap<>();
    static {
        //FLIPPER POSITIONS
        flipperPositionValues.put(Intake.flipperPositions.UP, intakeFlipperUp);
        flipperPositionValues.put(Intake.flipperPositions.PIXEL5, intakeFlipperPixel5);
        flipperPositionValues.put(Intake.flipperPositions.PIXEL4, intakeFlipperPixel4);
        flipperPositionValues.put(Intake.flipperPositions.PIXEL3, intakeFlipperPixel3);

    }
    public void setIntakeFlipperPosition(flipperPositions flipperPosition){
        intakeFlipperActionManger.setServoPosition(flipperPositionValues.get(flipperPosition));
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
                        flipperCase = flipperPositions.PIXEL5;
                    }
                    break;

                case PIXEL5:
                    if(handlerLeftStick_Down.Pressed()){
                        intakeFlipper.setPosition(intakeFlipperPixel4);
                        flipperCase = flipperPositions.PIXEL4;
                    }
                    if(handlerLeftStick_Up.Pressed()){
                        intakeFlipper.setPosition(intakeFlipperUp);
                        flipperCase = flipperPositions.UP;
                    }
                    break;

                case PIXEL4:
                    if(handlerLeftStick_Down.Pressed()){
                        intakeFlipper.setPosition(intakeFlipperPixel3);
                        flipperCase = flipperPositions.PIXEL3;
                    }
                    if(handlerLeftStick_Up.Pressed()){
                        intakeFlipper.setPosition(intakeFlipperPixel5);
                        flipperCase = flipperPositions.PIXEL5;
                    }
                    break;

                case PIXEL3:
                    if(handlerLeftStick_Up.Pressed()){
                        intakeFlipper.setPosition(intakeFlipperPixel4);
                        flipperCase = flipperPositions.PIXEL4;
                    }
                    if(handlerLeftStick_Down.Pressed()){
                        intakeFlipper.setPosition(intakeFlipperUp);
                        flipperCase = flipperPositions.UP;
                    }
                    break;
            }
        }
    }

}