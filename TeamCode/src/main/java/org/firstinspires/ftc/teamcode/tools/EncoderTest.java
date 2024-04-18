package org.firstinspires.ftc.teamcode.tools;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.roadRunner.util.Encoder;

@TeleOp(name="EncoderTest", group = "Tools")
public class EncoderTest extends LinearOpMode {

    Buttons buttons;
    DcMotor leftEncoder, rightEncoder, backEncoder;
    double RightEncoder, LeftEncoder, BackEncoder;
    public void Setup() {
        buttons = new Buttons(gamepad1, gamepad2);
        leftEncoder = hardwareMap.dcMotor.get("backLeftMotor");
        rightEncoder = hardwareMap.dcMotor.get("intakeMotor");
        backEncoder = hardwareMap.dcMotor.get("frontLeftMotor");

        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        RightEncoder = 0;
        LeftEncoder = 0;
        BackEncoder = 0;

    }
    @Override
    public void runOpMode() throws InterruptedException {
        Setup();
        waitForStart();

        while (opModeIsActive()) {
            if (buttons.handlerX.Pressed()) {
                leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                backEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            RightEncoder = rightEncoder.getCurrentPosition();
            LeftEncoder = leftEncoder.getCurrentPosition();
            BackEncoder = backEncoder.getCurrentPosition();

            telemetry.addData("Right encoder:", rightEncoder.getCurrentPosition());
            telemetry.addData("Left Encoder:", leftEncoder.getCurrentPosition());
            telemetry.addData("Back Encoder:", backEncoder.getCurrentPosition());
            telemetry.update();
        }
    }
}
