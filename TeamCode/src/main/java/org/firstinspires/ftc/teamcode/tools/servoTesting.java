package org.firstinspires.ftc.teamcode.tools;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "servoTesting", group = "Testing")
public class servoTesting extends LinearOpMode {
    // declare buttons

    Button handlerDPad_Left;
    Button handlerDPad_Up;
    Button handlerDPad_Down;
    Button handlerDPad_Right;
    Button handlerRightTrigger;
    Servo clawYaw;
    Servo clawPitch;
    Servo clawGrip;

    Servo droneAngle;

    AnalogInput armPositionInput;

    AnalogInput rotationPositionInput;

    DcMotor liftMotor;

    DcMotor hangerMotor;

    DcMotor hangerEncoder;

    DcMotor motor;
    DcMotor motor2;

    double clawYawIntake, clawYawLeftHorizontal, clawYawLeftSlantedUp, clawYawLeftSlantedDown,  clawYawRightHorizontal, clawYawRightSlantedUp, clawYawRightSlantedDown;
    double armPosition, rotationPosition, liftPosition, hangerPosition;

    ArrayList list;

    int hangerPower;

    public void Setup() {

        clawYaw = hardwareMap.servo.get("clawYaw");
        clawPitch = hardwareMap.servo.get("clawPitch");
        clawGrip = hardwareMap.servo.get("clawGrip");

        rotationPositionInput = hardwareMap.get(AnalogInput.class, "rotationPositionInput");
        armPositionInput = hardwareMap.get(AnalogInput.class, "armPositionInput");

        motor = hardwareMap.dcMotor.get("frontLeftMotor"); // for debugging
        motor2 = hardwareMap.dcMotor.get("backRightMotor"); // for debugging

        // map buttons
        handlerDPad_Left = new Button(gamepad2, Button.NAME.DPAD_LEFT);
        handlerDPad_Down = new Button(gamepad2, Button.NAME.DPAD_DOWN);
        handlerDPad_Up = new Button(gamepad2, Button.NAME.DPAD_UP);
        handlerDPad_Right = new Button(gamepad2, Button.NAME.DPAD_RIGHT);
        handlerRightTrigger = new Button(gamepad2, Button.NAME.RIGHT_TRIGGER);

        clawGrip.scaleRange(0.03, 0.25);
        //clawPitch.scaleRange(0.755, 0.950);
        clawPitch.scaleRange(0, 1);
        clawYaw.scaleRange(0, 1);

        liftMotor = (DcMotorEx) hardwareMap.dcMotor.get("liftMotor");
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        hangerMotor = (DcMotorEx) hardwareMap.dcMotor.get("skyHookMotor");
        hangerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        hangerEncoder = (DcMotorEx) hardwareMap.dcMotor.get("frontRightMotor");
        hangerEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        Setup();
        waitForStart();
        while (opModeIsActive()) {

            //motor.setPower(1);
            //motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            //motor2.setPower(1);
            //motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            hangerMotor.setPower(hangerPower);
            hangerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            liftPosition = liftMotor.getCurrentPosition();
            hangerPosition = hangerEncoder.getCurrentPosition();

            if (handlerDPad_Right.Pressed()){
                hangerMotor.setPower(-1);
                sleep(5);
                hangerMotor.setPower(0);
            }
            if (handlerDPad_Left.Pressed()){
                hangerMotor.setPower(1);
                sleep(5);
                hangerMotor.setPower(0);
            }

            clawYaw.setPosition(0.5);
            liftMotor.setTargetPosition(1500);
            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftMotor.setPower(1);

            clawPitch.setPosition(0.28);

            // From axon 4th position wire plugged into the analog port
            // See https://docs.axon-robotics.com/axon-servos/analog-jst-board
            armPosition = armPositionInput.getVoltage() / 3.3 * 360;

            // From axon 4th position wire plugged into the analog port
            // See https://docs.axon-robotics.com/axon-servos/analog-jst-board
            rotationPosition = rotationPositionInput.getVoltage() / 3.3 * 360;

            telemetry.addLine("Rotation position: " + Double.toString(rotationPosition));
            telemetry.addLine("Arm position: " + Double.toString(armPosition));
            telemetry.addLine("Lift position: " + Double.toString(liftPosition));
            telemetry.addLine("Skyhook position: " + Double.toString(hangerPosition));

            // Position read for arm at intake position
            // ~321-328 degrees

            // Position read for rotation at intake position
            // ~180 degrees

            // Position read for minimum lift position
            // ~1000 ticks

            // Position read for maximum lift position
            // ~ <3700

            // Position read for maximum skyhook position
            // ~ <10 900

            telemetry.update();

            updateButtons();

            /*if(handlerDPad_Left.Pressed()){
                if(handlerRightTrigger.Pressed()){
                    clawYaw.setPosition(clawYawRightHorizontal);
                }
                else{
                    clawYaw.setPosition(clawYawLeftHorizontal);
                }
            }

            if(handlerDPad_Down.Pressed()){
                if(handlerRightTrigger.Pressed()){
                    clawYaw.setPosition(clawYawRightSlantedUp);
                }
                else{
                    clawYaw.setPosition(clawYawLeftSlantedDown);
                }
            }

            if(handlerDPad_Up.Pressed()){
                if(handlerRightTrigger.Pressed()){
                    clawYaw.setPosition(clawYawRightSlantedDown);
                }
                else{
                    clawYaw.setPosition(clawYawLeftSlantedUp);

                }
            }

             */
        }

    }

    private void updateButtons() {
        handlerDPad_Left.updateButton(gamepad2);
        handlerDPad_Right.updateButton(gamepad2);
        handlerDPad_Up.updateButton(gamepad2);
        handlerDPad_Down.updateButton(gamepad2);
    }
}
