package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.AnalogInput;
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
        OPEN(1.0),
        OPEN_HALFWAY(0.5),
        CLOSE(0.15),
        CLOSE_ONE_PIXEL(0);

        private final double value;

        gripPositions(double value) {
            this.value = value;
        }
    }

    public void setGripPosition(gripPositions gripPosition) {
        clawGripServoActionManager.setServoPosition(gripPosition.value);
    }

    public enum pitchPositions{
        INTAKE(0),
        OUTTAKE(1),
        RESET(323);

        private final double value;

        pitchPositions(double value) {
            this.value = value;
        }
    }

    public void setPitchPosition(pitchPositions pitchPosition) {
        clawPitchServoActionManger.setServoPosition(pitchPosition.value);
    }

    public boolean waitForAnalogPitchSensorAtPosition (pitchPositions pitchPosition, double tolerance) {
        return clawPitchServoActionManger.waitForAnalogServoSensorAtPosition(pitchPosition.value, 3.3, tolerance);
    }

    public enum yawPositions{
        INTAKE(0.5),
        LEFT_SLANT_UP(1),
        LEFT_HORIZONTAL(LEFT_SLANT_UP.value-0.21),
        LEFT_SLANT_DOWN(LEFT_HORIZONTAL.value-0.21),
        RIGHT_SLANT_UP(0),
        RIGHT_HORIZONTAL(RIGHT_SLANT_UP.value+0.21),
        RIGHT_SLANT_DOWN(RIGHT_HORIZONTAL.value+0.21),
        RESET(180);

        private final double value;

        yawPositions(double value) {
            this.value = value;
        }
    }

    public void setYawPosition(yawPositions yawPosition) {
        clawYaw.setPosition(yawPosition.value);
    }

    public boolean waitForAnalogYawSensorAtPosition (yawPositions yawPosition, double tolerance) {
        return clawYawServoActionManager.waitForAnalogServoSensorAtPosition(yawPosition.value, 3.3, tolerance);
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
