package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.tools.Global;
import org.firstinspires.ftc.teamcode.tools.MotorActionManager;

public class Lift {
    public MotorActionManager liftMotorActionManager;
    public DcMotorEx liftMotor;
    public TouchSensor liftTouchDown;
    Gamepad gamepad2;

    private final double handlerLiftDeadzone = 0.05;

    public enum Position {
        HOLDING(800),
        HOLDING_TELEOP(50),
        MIN(1400),
        MAX(3600),
        AUTO_MIN_YELLOW(MIN.value - 100),
        AUTO_MIN_YELLOW_LOW(150),
        AUTO_MIN_YELLOW_HIGH(MIN.value + 300);

        private final int value;

        Position(int value) {
            this.value = value;
        }

    }

    public Lift(HardwareMap hardwareMap, Gamepad gamepad2){
        liftMotor = (DcMotorEx) hardwareMap.dcMotor.get("liftMotor");
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftTouchDown = hardwareMap.touchSensor.get("liftTouchDown");


        liftMotorActionManager = new MotorActionManager(liftMotor);
        this.gamepad2 = gamepad2;
    }

    @SuppressWarnings("unboxing")
    public void setLiftPowerBasedOnGamepad(Gamepad gamepad) {
        double power = -gamepad.right_stick_y;
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if ((liftMotor.getCurrentPosition() > Position.MAX.value*1.05) && power > 0){
            power = 0;
        }
        if ((liftMotor.getCurrentPosition() < Position.MIN.value*0.95) && power < 0){
            power = 0;
        }

        if (power < -handlerLiftDeadzone || power > handlerLiftDeadzone){
            liftMotor.setPower(power);
        }
        else {
            liftMotor.setPower(0);
        }

    }

    public int getCurrentLiftMotorPosition(){
        return liftMotorActionManager.getCurrentMotorPosition();
    }

    public void holdPosition(int position) {
        liftMotorActionManager.holdPosition(position, 0.7);
    }

    public void setLiftPosition(Position position, double power) {
        liftMotorActionManager.holdPosition(position.value, power);
    }

    public boolean waitLiftMotorAbovePosition(Position expectedPosition) {
        return liftMotorActionManager.waitMotorAbovePosition(expectedPosition.value);
    }

    public boolean waitLiftMotorBelowPosition(int expectedPosition) {
        return liftMotorActionManager.waitMotorBelowPosition(expectedPosition);
    }

    public void resetLiftMotorEncoder() {
        liftMotorActionManager.resetMotorEncoder();
    }

    public void startLiftMotorWithEncoder(double power) {
        liftMotorActionManager.startMotorEncoder(power);
    }
    public void startLiftMotorWithNoEncoder(double power) {
        liftMotorActionManager.startMotorNoEncoder(power);
    }

    public void stopLiftMotor() {
        liftMotorActionManager.stopMotor();
    }

    public boolean liftTouchDownPressed(){
        return liftTouchDown.isPressed();
    }

    public void update() {

        // Bit of a hack (so that when it comes from state machine target pos is not 0)
        int lastSetPosition = Range.clip(liftMotor.getTargetPosition(), Position.MIN.value, Position.MAX.value);
        // If the handler does not want to move motor, then hold the last position set by the handler
        // Else, move the motor normally
        if ((Math.abs(gamepad2.right_stick_y)<handlerLiftDeadzone) ||
                (-gamepad2.right_stick_y>0 && lastSetPosition ==Position.MAX.value) ||
                (-gamepad2.right_stick_y<0 && lastSetPosition ==Position.MIN.value)){
            holdPosition(lastSetPosition);
        }
        else{
            setLiftPowerBasedOnGamepad(gamepad2);
            lastSetPosition = liftMotor.getCurrentPosition();

            // Hack continued :-)
            liftMotor.setTargetPosition(Range.clip(lastSetPosition, Position.MIN.value, Position.MAX.value));
        }
    }

}
