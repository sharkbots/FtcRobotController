package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.HashMap;
import java.util.Map;

public class Claw {

    public Claw(HardwareMap hardwareMap, Gamepad gamepad2) {
        clawPitch = hardwareMap.servo.get("clawPitch");
        clawYaw = hardwareMap.servo.get("clawYaw");
        clawGrip = hardwareMap.servo.get("clawGrip");
        clawYawAnalogSensor = hardwareMap.get(AnalogInput.class, "rotationPositionInput");
        clawPitchAnalogSensor = hardwareMap.get(AnalogInput.class, "armPositionInput");

        clawGrip.scaleRange(0, 0.23);
        clawPitch.scaleRange(0.07, 0.28);
        clawYaw.scaleRange(0, 1);

        this.gamepad2 = gamepad2;
    }

    Gamepad gamepad2;
    public static Servo clawPitch, clawYaw, clawGrip;
    public static AnalogInput clawYawAnalogSensor, clawPitchAnalogSensor;


    //GRIP
    private static double clawOpen = 1.0;
    private static double clawClose = 0.15;
    private static double clawCloseOnePixel = 0;
    public enum gripPositions {
        OPEN,
        CLOSE,
        CLOSE_ONE_PIXEL,
    }
    private static final Map<Claw.gripPositions, Double> gripPositionValues = new HashMap<>();
    static {
        gripPositionValues.put(gripPositions.OPEN, clawOpen);
        gripPositionValues.put(gripPositions.CLOSE, clawClose);
        gripPositionValues.put(gripPositions.CLOSE_ONE_PIXEL, clawCloseOnePixel);
    }
    public void setGripPosition(Claw.gripPositions gripPosition) {
        if (gripPositionValues.containsKey(gripPosition)) {
            clawGrip.setPosition(gripPositionValues.get(gripPosition));
        }
    }
    //PITCH
    private static double clawPitchIntake = 0;
    private static double clawPitchOutTake = 1;
    private static double analog_ClawPitch_ResetPosition = 323;
    public enum pitchPositions{
        INTAKE,
        OUTTAKE,
        RESET
    }
    private static final Map<Claw.pitchPositions, Double> pitchPositionValues = new HashMap<>();
    static {
        pitchPositionValues.put(pitchPositions.INTAKE, clawPitchIntake);
        pitchPositionValues.put(pitchPositions.OUTTAKE, clawPitchOutTake);
        pitchPositionValues.put(pitchPositions.RESET, analog_ClawPitch_ResetPosition);
    }
    public void setPitchPosition(Claw.pitchPositions pitchPosition) {
        if (pitchPositionValues.containsKey(pitchPosition)) {
            clawPitch.setPosition(pitchPositionValues.get(pitchPosition));
        }
    }

    public boolean waitForAnalogPitchSensorAtPosition () {
        double currentPosition = clawPitchAnalogSensor.getVoltage() / 3.3 * 360;
        return Math.abs(analog_ClawPitch_ResetPosition - currentPosition) <= 10;
    }

//YAW
    private static double clawYawIntake = 0.5;
    // Slanted is 60 degrees, allows us to drop pixels vertically for mosaics
    private static double clawYawLeftSlantedUp = 1;
    private static double clawYawLeftHorizontal = clawYawLeftSlantedUp-0.21;
    private static double clawYawLeftSlantedDown = clawYawLeftHorizontal-0.21;

    private static double clawYawRightSlantedUp = 0;
    private static double clawYawRightHorizontal = clawYawRightSlantedUp+0.21;
    private static double clawYawRightSlantedDown = clawYawRightHorizontal+0.21;
    private static double analog_ClawYaw_ResetPosition = 180;

    public enum yawPositions{
        INTAKE,
        LEFT_SLANT_UP,
        LEFT_HORIZONTAL,
        LEFT_SLANT_DOWN,
        RIGHT_SLANT_UP,
        RIGHT_HORIZONTAL,
        RIGHT_SLANT_DOWN,
        RESET
    }
    private static final Map<Claw.yawPositions, Double> yawPositionValues = new HashMap<>();
    static {
        yawPositionValues.put(yawPositions.INTAKE, clawYawIntake);
        yawPositionValues.put(yawPositions.LEFT_SLANT_UP, clawYawLeftSlantedUp);
        yawPositionValues.put(yawPositions.LEFT_HORIZONTAL, clawYawLeftHorizontal);
        yawPositionValues.put(yawPositions.LEFT_SLANT_DOWN, clawYawLeftSlantedDown);
        yawPositionValues.put(yawPositions.RIGHT_SLANT_UP, clawYawRightSlantedUp);
        yawPositionValues.put(yawPositions.RIGHT_HORIZONTAL, clawYawRightHorizontal);
        yawPositionValues.put(yawPositions.RIGHT_SLANT_DOWN, clawYawRightSlantedDown);
        yawPositionValues.put(yawPositions.RESET, analog_ClawYaw_ResetPosition);
    }
    public void setYawPosition(Claw.yawPositions yawPosition) {
        if (yawPositionValues.containsKey(yawPosition)) {
            clawYaw.setPosition(yawPositionValues.get(yawPosition));
        }
    }

    public boolean waitForAnalogYawSensorAtPosition () {
        double currentPosition = clawYawAnalogSensor.getVoltage() / 3.3 * 360;
        return Math.abs(analog_ClawYaw_ResetPosition - currentPosition) <= 5;
    }

}
