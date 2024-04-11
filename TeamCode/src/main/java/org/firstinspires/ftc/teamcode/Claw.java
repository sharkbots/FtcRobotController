package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.tools.Buttons;
import org.firstinspires.ftc.teamcode.tools.Robot;
import org.firstinspires.ftc.teamcode.tools.ServoActionManager;

public class Claw {

    public Claw(HardwareMap hardwareMap, Buttons buttons) {
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

        this.buttons = buttons;
    }

    Buttons buttons;
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
        if(buttons.handlerDPad_Left.Pressed()){
            if(buttons.handlerRightTrigger.On()){
                Robot.claw.setYawPosition(Claw.yawPositions.RIGHT_HORIZONTAL);
                //Robot.clawYaw.setPosition(Robot.clawYawRightHorizontal);
            }
            else{
                Robot.claw.setYawPosition(Claw.yawPositions.LEFT_HORIZONTAL);
                //Robot.clawYaw.setPosition(Robot.clawYawLeftHorizontal);
            }
        }

        if(buttons.handlerDPad_Down.Pressed()){
            if(buttons.handlerRightTrigger.On()){
                Robot.claw.setYawPosition(Claw.yawPositions.RIGHT_SLANT_UP);
                //Robot.clawYaw.setPosition(Robot.clawYawRightSlantedUp);
            }
            else{
                Robot.claw.setYawPosition(Claw.yawPositions.LEFT_SLANT_DOWN);
                //Robot.clawYaw.setPosition(Robot.clawYawLeftSlantedDown);
            }
        }

        if(buttons.handlerDPad_Up.Pressed()){
            if(buttons.handlerRightTrigger.On()){
                Robot.claw.setYawPosition(Claw.yawPositions.RIGHT_SLANT_DOWN);
                //Robot.clawYaw.setPosition(Robot.clawYawRightSlantedDown);
            }
            else{
                Robot.claw.setYawPosition(Claw.yawPositions.LEFT_SLANT_UP);
                //Robot.clawYaw.setPosition(Robot.clawYawLeftSlantedUp);

            }
        }

        if(buttons.handlerDPad_Right.Pressed()){
            if(buttons.handlerRightTrigger.On()){
                //Robot.clawYaw.setPosition(Robot.clawYawIntake);
                Robot.claw.setYawPosition(Claw.yawPositions.INTAKE);
            }
            else{
                Robot.claw.setYawPosition(Claw.yawPositions.INTAKE);
                //Robot.clawYaw.setPosition(Robot.clawYawIntake);
            }
        }


        if(buttons.handlerRightBumper.Pressed()){
            Robot.claw.setGripPosition(Claw.gripPositions.OPEN);
            //Robot.clawGrip.setPosition(Robot.clawOpen);
        }
        if(buttons.handlerLeftBumper.Pressed()){
            Robot.claw.setGripPosition(Claw.gripPositions.CLOSE);
            //Robot.clawGrip.setPosition(Robot.clawClose);
        }

    }

}
