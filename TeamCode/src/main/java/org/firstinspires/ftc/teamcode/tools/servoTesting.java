package org.firstinspires.ftc.teamcode.tools;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

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

    DcMotor backRightMotor;

    double clawYawIntake, clawYawLeftHorizontal, clawYawLeftSlantedUp, clawYawLeftSlantedDown,  clawYawRightHorizontal, clawYawRightSlantedUp, clawYawRightSlantedDown;
    double armPosition, rotationPosition, liftPosition, hangerPosition;

    int hangerPower;

    public void Setup() {

        clawYaw = hardwareMap.servo.get("clawYaw");
        clawPitch = hardwareMap.servo.get("clawPitch");
        clawGrip = hardwareMap.servo.get("clawGrip");
        droneAngle = hardwareMap.servo.get("droneAngle");

        rotationPositionInput = hardwareMap.get(AnalogInput.class, "rotationPositionInput");
        armPositionInput = hardwareMap.get(AnalogInput.class, "armPositionInput");

        // map buttons
        handlerDPad_Left = new Button(gamepad2, Button.NAME.DPAD_LEFT);
        handlerDPad_Down = new Button(gamepad2, Button.NAME.DPAD_DOWN);
        handlerDPad_Up = new Button(gamepad2, Button.NAME.DPAD_UP);
        handlerDPad_Right = new Button(gamepad2, Button.NAME.DPAD_RIGHT);
        handlerRightTrigger = new Button(gamepad2, Button.NAME.RIGHT_TRIGGER);

        clawGrip.scaleRange(0.03, 0.25);
        //clawPitch.scaleRange(0.755, 0.950);
        clawPitch.scaleRange(0.07, 0.25);
        clawYaw.scaleRange(0, 1);

        liftMotor = (DcMotorEx) hardwareMap.dcMotor.get("liftMotor");
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        hangerMotor = (DcMotorEx) hardwareMap.dcMotor.get("hangerMotor");
        hangerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        backRightMotor = (DcMotorEx) hardwareMap.dcMotor.get("backRightMotor");
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        Setup();
        waitForStart();
        while (opModeIsActive()) {
            hangerMotor.setPower(hangerPower);
            hangerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            liftPosition = liftMotor.getCurrentPosition();
            hangerPosition = backRightMotor.getCurrentPosition();

            if (handlerDPad_Up.On()){
                hangerPower = 1;
                telemetry.addLine("hello");
            }
            else if (handlerDPad_Down.On()){
                hangerPower = -1;
            }
            else {
                hangerPower = 0;
            }

            if (handlerDPad_Right.Pressed()){
                hangerMotor.setPower(1);
                sleep(5);
            }
            if (handlerDPad_Left.Pressed()){
                hangerMotor.setPower(-1);
                sleep(5);
            }

            if (hangerPosition >= 10800 && hangerPower > 0){
                hangerPower = 0;
            }

            if (hangerPosition < 100 && hangerPower < 0){
                hangerPower = 0;
            }

            clawPitch.setPosition(0);
            clawYaw.setPosition(0.5);

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
