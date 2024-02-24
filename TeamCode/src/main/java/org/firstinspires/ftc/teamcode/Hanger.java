package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.tools.Button;

public class Hanger {
    private final DcMotor hangerMotor;
    private final DcMotor hangerEncoder;
    private final int countsPerRev = 1531; //not sure of this value
    private final double HANGER_REVOLUTIONS = 0.4; //need to test to see how many revolutions
    private final Button handlerDPadDown, handlerDPadUp;

    private final int maxSkyHookPosition = 10800;
    private final int minSkyHookPosition = 100;


    public Hanger(HardwareMap hardwareMap, Button handlerDPadDown, Button handlerDPadUp) {
        hangerMotor = hardwareMap.dcMotor.get("skyHookMotor");
        hangerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Hanger encoder is plugged into a different port than the hanger motor
        hangerEncoder = hardwareMap.dcMotor.get("backRightMotor");
        //hangerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        this.handlerDPadDown = handlerDPadDown;
        this.handlerDPadUp = handlerDPadUp;
    }

    public void update(Button button) {
        //TelemetryManager.getTelemetry().addData("Hanger Pos: ", hangerMotor.getCurrentPosition());
        if(handlerDPadUp.On()){
            if(hangerEncoder.getCurrentPosition() >= maxSkyHookPosition) {
                hangerMotor.setPower(0);
            }
            else {
                //int targetPosition = (int) (countsPerRev * HANGER_REVOLUTIONS); // cast to int
                /*hangerMotor.setTargetPosition(targetPosition);
                hangerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);*/
                hangerMotor.setPower(1);
            }
        }
        else if(handlerDPadDown.On()){ // button is pressed
            //int targetPosition = (int)(countsPerRev * HANGER_REVOLUTIONS); // cast to int
            //hangerMotor.setTargetPosition(0);
            //hangerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            if(hangerEncoder.getCurrentPosition() <= minSkyHookPosition) {
                hangerMotor.setPower(0);
            }
            else {
                hangerMotor.setPower(-1);
                //hangerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                //hangerMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }
    }

}
