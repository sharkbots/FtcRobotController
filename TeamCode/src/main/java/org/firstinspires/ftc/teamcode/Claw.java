package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.tools.Robot;
import org.firstinspires.ftc.teamcode.tools.ServoActionManager;

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

        clawPitchServoActionManger = new ServoActionManager(clawPitch, clawPitchAnalogSensor);
        clawYawServoActionManager = new ServoActionManager(clawYaw, clawYawAnalogSensor);
        clawGripServoActionManager = new ServoActionManager(clawGrip);

        this.gamepad2 = gamepad2;
    }

    Gamepad gamepad2;
    public static Servo clawPitch, clawYaw, clawGrip;
    public static AnalogInput clawYawAnalogSensor, clawPitchAnalogSensor;
    public static ServoActionManager clawPitchServoActionManger, clawYawServoActionManager, clawGripServoActionManager;

    public enum gripPositions {
        OPEN,
        OPEN_AUTO,
        CLOSE,
        CLOSE_ONE_PIXEL,
    }
    private static final Map<Claw.gripPositions, Double> gripPositionValues = new HashMap<>();
    static {
        //GRIP
        double clawOpen = 1.0;
        gripPositionValues.put(gripPositions.OPEN, clawOpen);

        double clawOpenAuto = 0.5;
        gripPositionValues.put(gripPositions.OPEN_AUTO, clawOpenAuto);


        double clawClose = 0.15;
        gripPositionValues.put(gripPositions.CLOSE, clawClose);

        double clawCloseOnePixel = 0;
        gripPositionValues.put(gripPositions.CLOSE_ONE_PIXEL, clawCloseOnePixel);
    }
    public void setGripPosition(gripPositions gripPosition) {
        if (gripPositionValues.containsKey(gripPosition)) {
            clawGripServoActionManager.setServoPosition(gripPositionValues.get(gripPosition));
        }
    }

    public enum pitchPositions{
        INTAKE,
        OUTTAKE,
        RESET
    }
    private static final Map<Claw.pitchPositions, Double> pitchPositionValues = new HashMap<>();
    static {
        //PITCH
        double clawPitchIntake = 0;
        pitchPositionValues.put(pitchPositions.INTAKE, clawPitchIntake);

        double clawPitchOutTake = 1;
        pitchPositionValues.put(pitchPositions.OUTTAKE, clawPitchOutTake);

        double analog_ClawPitch_ResetPosition = 323;
        pitchPositionValues.put(pitchPositions.RESET, analog_ClawPitch_ResetPosition);
    }
    public void setPitchPosition(pitchPositions pitchPosition) {
        if (pitchPositionValues.containsKey(pitchPosition)) {
            clawPitchServoActionManger.setServoPosition(pitchPositionValues.get(pitchPosition));
        }
    }

    public boolean waitForAnalogPitchSensorAtPosition (pitchPositions pitchPosition, double tolerance) {
        return clawPitchServoActionManger.waitForAnalogServoSensorAtPosition(pitchPositionValues.get(pitchPosition), 3.3, tolerance);
    }

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
        //YAW
        // Slanted is 60 degrees, allows us to drop pixels vertically for mosaics

        double clawYawIntake = 0.5;
        yawPositionValues.put(yawPositions.INTAKE, clawYawIntake);

        double clawYawLeftSlantedUp = 1;
        yawPositionValues.put(yawPositions.LEFT_SLANT_UP, clawYawLeftSlantedUp);

        double clawYawLeftHorizontal = clawYawLeftSlantedUp-0.21;
        yawPositionValues.put(yawPositions.LEFT_HORIZONTAL, clawYawLeftHorizontal);

        double clawYawLeftSlantedDown = clawYawLeftHorizontal-0.21;
        yawPositionValues.put(yawPositions.LEFT_SLANT_DOWN, clawYawLeftSlantedDown);

        double clawYawRightSlantedUp = 0;
        yawPositionValues.put(yawPositions.RIGHT_SLANT_UP, clawYawRightSlantedUp);

        double clawYawRightHorizontal = clawYawRightSlantedUp+0.21;
        yawPositionValues.put(yawPositions.RIGHT_HORIZONTAL, clawYawRightHorizontal);

        double clawYawRightSlantedDown = clawYawRightHorizontal+0.21;
        yawPositionValues.put(yawPositions.RIGHT_SLANT_DOWN, clawYawRightSlantedDown);

        double analog_ClawYaw_ResetPosition = 180;
        yawPositionValues.put(yawPositions.RESET, analog_ClawYaw_ResetPosition);
    }
    public void setYawPosition(yawPositions yawPosition) {
        if (yawPositionValues.containsKey(yawPosition)) {
            clawYaw.setPosition(yawPositionValues.get(yawPosition));
        }
    }

    public boolean waitForAnalogYawSensorAtPosition (yawPositions yawPosition, double tolerance) {
        return clawYawServoActionManager.waitForAnalogServoSensorAtPosition(yawPositionValues.get(yawPosition), 3.3, tolerance);
    }

    public void update(){
        //telemetry.addData("Requested position: ", Robot.clawYaw.getPosition());
        if(Robot.handlerDPad_Left.Pressed()){
            if(Robot.handlerRightTrigger.On()){
                Robot.claw.setYawPosition(Claw.yawPositions.RIGHT_HORIZONTAL);
                //Robot.clawYaw.setPosition(Robot.clawYawRightHorizontal);
            }
            else{
                Robot.claw.setYawPosition(Claw.yawPositions.LEFT_HORIZONTAL);
                //Robot.clawYaw.setPosition(Robot.clawYawLeftHorizontal);
            }
        }

        if(Robot.handlerDPad_Down.Pressed()){
            if(Robot.handlerRightTrigger.On()){
                Robot.claw.setYawPosition(Claw.yawPositions.RIGHT_SLANT_UP);
                //Robot.clawYaw.setPosition(Robot.clawYawRightSlantedUp);
            }
            else{
                Robot.claw.setYawPosition(Claw.yawPositions.LEFT_SLANT_DOWN);
                //Robot.clawYaw.setPosition(Robot.clawYawLeftSlantedDown);
            }
        }

        if(Robot.handlerDPad_Up.Pressed()){
            if(Robot.handlerRightTrigger.On()){
                Robot.claw.setYawPosition(Claw.yawPositions.RIGHT_SLANT_DOWN);
                //Robot.clawYaw.setPosition(Robot.clawYawRightSlantedDown);
            }
            else{
                Robot.claw.setYawPosition(Claw.yawPositions.LEFT_SLANT_UP);
                //Robot.clawYaw.setPosition(Robot.clawYawLeftSlantedUp);

            }
        }

        if(Robot.handlerDPad_Right.Pressed()){
            if(Robot.handlerRightTrigger.On()){
                //Robot.clawYaw.setPosition(Robot.clawYawIntake);
                Robot.claw.setYawPosition(Claw.yawPositions.INTAKE);
            }
            else{
                Robot.claw.setYawPosition(Claw.yawPositions.INTAKE);
                //Robot.clawYaw.setPosition(Robot.clawYawIntake);
            }
        }


        if(Robot.handlerRightBumper.Pressed()){
            Robot.claw.setGripPosition(Claw.gripPositions.OPEN);
            //Robot.clawGrip.setPosition(Robot.clawOpen);
        }
        if(Robot.handlerLeftBumper.Pressed()){
            Robot.claw.setGripPosition(Claw.gripPositions.CLOSE);
            //Robot.clawGrip.setPosition(Robot.clawClose);
        }

    }

}
