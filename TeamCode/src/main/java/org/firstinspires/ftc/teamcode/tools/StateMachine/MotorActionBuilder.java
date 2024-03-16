package org.firstinspires.ftc.teamcode.tools.StateMachine;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.tools.Robot;

public class MotorActionBuilder {
    MotorActionBuilder(DcMotor singleMotor) {
        this.motor = singleMotor;
    }

    Action resetMotorEncoder() {
        ActionFunction function = () -> {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            return true;
        };
        String name =
                "stop and reset " + motor.getDeviceName();
        return new Action(name, function);
    }

    Action stopMotor() {
        ActionFunction function = () -> {
            motor.setPower(0);
            return true;
        };
        String name =
                "stop motor " + motor.getDeviceName();
        return new Action(name, function);
    }

    private DcMotor motor;
}
