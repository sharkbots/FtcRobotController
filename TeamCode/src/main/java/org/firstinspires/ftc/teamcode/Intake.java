package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.tools.Button;
import org.firstinspires.ftc.teamcode.tools.OverrideMotor;

public class Intake {
    public final OverrideMotor intakeMotor;
    private final  Button handlerLeftTrigger;

    public Intake(HardwareMap hardwareMap, Button handlerLeftTrigger) {
        // Motors
        intakeMotor = new OverrideMotor(hardwareMap.dcMotor.get("intakeMotor"));
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        this.handlerLeftTrigger = handlerLeftTrigger;
    }

    public void update(){
        // Manages Reject mode on Roomba as an override of its current power and state
        if(handlerLeftTrigger.Pressed()) {
            intakeMotor.setOverridePower(-1);
        } else if (handlerLeftTrigger.Released()) {
            intakeMotor.cancelOverridePower();
        }
    }
}
