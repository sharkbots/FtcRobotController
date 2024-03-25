package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.tools.Global;
import org.firstinspires.ftc.teamcode.tools.MotorActionManager;

import java.util.HashMap;
import java.util.Map;

public class Lift {
    private static final Map<liftPositions, Integer> liftPositionValues = new HashMap<>();
    public MotorActionManager liftMotorActionManager;
    public DcMotorEx liftMotor;
    public TouchSensor liftTouchDown;
    Gamepad gamepad2;

    private final double handlerLiftDeadzone = 0.05;

    public enum liftPositions {
        HOLDING,
        HOLDING_TELEOP,

        MIN,
        MAX,
        AUTO_MIN_YELLOW,
        AUTO_MIN_YELLOW_LOW,
        AUTO_MIN_YELLOW_HIGH
    }
    static {
        int liftEncoderHolding = 800;
        liftPositionValues.put(liftPositions.HOLDING, liftEncoderHolding);

        int liftEncoderHoldingTeleop = 50;
        liftPositionValues.put(liftPositions.HOLDING_TELEOP, liftEncoderHoldingTeleop);

        int liftEncoderHoldingLow = 150;
        liftPositionValues.put(liftPositions.AUTO_MIN_YELLOW_LOW, liftEncoderHoldingLow);

        int liftEncoderMin = 1400;
        liftPositionValues.put(liftPositions.MIN, liftEncoderMin);

        int liftEncoderMax = 3600;
        liftPositionValues.put(liftPositions.MAX, liftEncoderMax);

        int liftEncoderMinYellowLow = liftEncoderMin-100;
        liftPositionValues.put(liftPositions.AUTO_MIN_YELLOW, liftEncoderMinYellowLow);

        int liftEncoderMinYellowHigh = liftEncoderMin+300;
        liftPositionValues.put(liftPositions.AUTO_MIN_YELLOW_HIGH, liftEncoderMinYellowHigh);
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

    public Lift(HardwareMap hardwareMap){
        liftMotor = (DcMotorEx) hardwareMap.dcMotor.get("liftMotor");
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftTouchDown = hardwareMap.touchSensor.get("liftTouchDown");


        liftMotorActionManager = new MotorActionManager(liftMotor);
        this.gamepad2 = null;
    }

    public void setLiftPowerBasedOnGamepad(Gamepad gamepad) {
        double power = -gamepad.right_stick_y;
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if ((liftMotor.getCurrentPosition() > liftPositionValues.get(liftPositions.MAX)*1.05) && power > 0){
            power = 0;
        }
        if ((liftMotor.getCurrentPosition() < liftPositionValues.get(liftPositions.MIN)*0.95) && power < 0){
            power = 0;
        }

        if (power < -handlerLiftDeadzone || power > handlerLiftDeadzone){
            liftMotor.setPower(power);
        }
        else {
            liftMotor.setPower(0);
            power = 0;
        }

    }

    public int getCurrentLiftMotorPosition(){
        return liftMotorActionManager.getCurrentMotorPosition();
    }

    public void holdPosition(int position) {
        liftMotorActionManager.holdPosition(position, 0.7);
    }

    public void setLiftPosition(liftPositions liftPosition, double power) {
        if (liftPositionValues.containsKey(liftPosition)) {
            liftMotorActionManager.holdPosition(liftPositionValues.get(liftPosition), power);
        }
    }

    public boolean waitLiftMotorAbovePosition(liftPositions expectedPosition) {
        return liftMotorActionManager.waitMotorAbovePosition(liftPositionValues.get(expectedPosition));
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
    private int lastSetPosition = 0;

    public void update() {

        // Bit of a hack (so that when it comes from state machine target pos is not 0)
        lastSetPosition = Global.ensureWithin(liftMotor.getTargetPosition(), liftPositionValues.get(liftPositions.MIN), liftPositionValues.get(liftPositions.MAX));

        // If the handler does not want to move motor, then hold the last position set by the handler
        // Else, move the motor normally
        if ((Math.abs(gamepad2.right_stick_y)<handlerLiftDeadzone) ||
                (-gamepad2.right_stick_y>0 && lastSetPosition ==liftPositionValues.get(liftPositions.MAX)) ||
                (-gamepad2.right_stick_y<0 && lastSetPosition ==liftPositionValues.get(liftPositions.MIN))

        ){
            holdPosition(lastSetPosition);
        }
        else{
            setLiftPowerBasedOnGamepad(gamepad2);
            lastSetPosition = liftMotor.getCurrentPosition();

            // Hack continued :-)
            liftMotor.setTargetPosition(Global.ensureWithin(lastSetPosition, liftPositionValues.get(liftPositions.MIN), liftPositionValues.get(liftPositions.MAX)));
        }
    }

}
