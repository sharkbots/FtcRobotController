package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.tools.Button;
import org.firstinspires.ftc.teamcode.tools.Global;

public class Hanger {
    private final DcMotor hangerMotor;
    private final DcMotor hangerEncoder;
    private final int countsPerRev = 1531; //not sure of this value
    private final double HANGER_REVOLUTIONS = 0.4; //need to test to see how many revolutions
    private final Button handlerDPadDown, handlerDPadUp, handlerY;

    private final int maxSkyHookPosition = 3600;
    private final int minSkyHookPosition = 100;

    private double hangerPower = 0;

    private int autoMoveHanger = 0; // 0 means nothing, 1 means up, -1 means down for the auto go up


    public Hanger(HardwareMap hardwareMap, Button handlerDPadDown, Button handlerDPadUp, Button handlerY) {
        hangerMotor = hardwareMap.dcMotor.get("skyHookMotor");
        hangerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hangerMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Hanger encoder is plugged into a different port than the hanger motor
        hangerEncoder = hardwareMap.dcMotor.get("frontRightMotor");
        hangerEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangerEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // ugh this took 2 hours to debug. I over-rode the other drive mode oops

        this.handlerDPadDown = handlerDPadDown;
        this.handlerDPadUp = handlerDPadUp;
        this.handlerY = handlerY;
    }

    public void update() {
        //Global.telemetry.addLine("Skyhook position is: " + hangerEncoder.getCurrentPosition());
        //Global.telemetry.update();

        if(handlerDPadUp.On()){
            autoMoveHanger = 0;
            hangerPower = 1;
        }
        else if(handlerDPadDown.On()){ // button is pressed
            autoMoveHanger = 0;
            hangerPower = -1;
        }
        else if (autoMoveHanger == 0){ // We stop the motor only in manual mode
            hangerPower = 0;
        }

        if(handlerY.Pressed()){
            if (autoMoveHanger == 0){
                autoMoveHanger = -1; // We're flipping the value each time. We start with -1
            }
            autoMoveHanger = -autoMoveHanger;
            hangerPower = autoMoveHanger;
        }
        if (hangerPower < 0 && hangerEncoder.getCurrentPosition() <= minSkyHookPosition) {
            hangerPower = 0;
        }
        else if (hangerPower > 0 && hangerEncoder.getCurrentPosition() >= maxSkyHookPosition){
            hangerPower = 0;
        }

        hangerMotor.setPower(hangerPower);
    }
}
