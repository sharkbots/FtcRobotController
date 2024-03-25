package org.firstinspires.ftc.teamcode.tools;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class MotorActionManager {
    private final DcMotorEx motor;


    public MotorActionManager(DcMotorEx motor) {
        this.motor = motor;
    }

    public MotorActionManager(OverrideMotor motor){
        this.motor = motor;
    }

    public void holdPosition(int position, double power){
        motor.setTargetPosition(position);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(power);
    }
    public boolean waitMotorAbovePosition(int expectedPosition) {
        return motor.getCurrentPosition() >= expectedPosition*0.95;
    }

    public boolean waitMotorBelowPosition(int expectedPosition) {
        return motor.getCurrentPosition() <= expectedPosition*0.95;
    }

    public void resetMotorEncoder() {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }


    public void startMotorEncoder(double power) {
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setPower(power);
    }

    public void startMotorNoEncoder(double power) {
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setPower(power);
    }

    public void stopMotor() {
        motor.setPower(0);
    }

    public int getCurrentMotorPosition(){
        return motor.getCurrentPosition();
    }
}
