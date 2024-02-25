package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.tools.Global;

public class Lift {

    public final int liftEncoderHolding = 800; // Set your minimum encoder value here
    public final int liftEncoderHoldingTeleop = 50; // Set your minimum encoder value here
    public final int liftEncoderMin = 1600; // Set your minimum encoder value here
    private final int liftEncoderMax = 3600;
    public DcMotorEx liftMotor;
    Gamepad gamepad2;

    private final double handlerLiftDeadzone = 0.05;

    private int lastSetPosition = 0;


    public Lift(HardwareMap hardwareMap, Gamepad gamepad2){
        liftMotor = (DcMotorEx) hardwareMap.dcMotor.get("liftMotor");
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.gamepad2 = gamepad2;
    }

    public void update() {

        // Bit of a monkey patch (so that when it comes from state machine target pos is not 0)
        lastSetPosition = liftMotor.getTargetPosition();

        // If the handler does not want to move motor, then hold the last position set by the handler
        // Else, move the motor normally
        if (Math.abs(gamepad2.right_stick_y)<handlerLiftDeadzone){
            // Makes sure lift does not go past its limits
            if (lastSetPosition < liftEncoderMin){
                lastSetPosition = liftEncoderMin;
            }
            else if (lastSetPosition > liftEncoderMax){
                lastSetPosition = liftEncoderMax;
            }

            holdPosition(lastSetPosition);
        }
        else{
            setLiftPowerBasedOnGamepad(gamepad2);
            lastSetPosition = liftMotor.getCurrentPosition();

            // Monkey patch continued :-)
            liftMotor.setTargetPosition(lastSetPosition);
        }

        Global.telemetry.addData("right_stick_y: ", gamepad2.right_stick_y);
        Global.telemetry.update();
    }

    public void setLiftPowerBasedOnGamepad(Gamepad gamepad) {
        double power = -gamepad.right_stick_y;
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if ((liftMotor.getCurrentPosition() > liftEncoderMax*0.95) && power > 0){
            power = 0;
        }
        if ((liftMotor.getCurrentPosition() < liftEncoderMin*0.95) && power < 0){
            power = 0;
        }

        if (power > handlerLiftDeadzone || power < -handlerLiftDeadzone){
            liftMotor.setPower(power);
        }
        else {
            liftMotor.setPower(0);
        }

        Global.telemetry.addData("power: ", power);
        Global.telemetry.addData("right_stick_y: ", gamepad.right_stick_y);
        Global.telemetry.addData("position: ", liftMotor.getCurrentPosition());
        Global.telemetry.update();
    }

    public void holdPosition(int position) {
        liftMotor.setTargetPosition(position);
        //liftMotor.setTargetPositionTolerance(100);
        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotor.setPower(0.7);
    }
}
