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

    private DcMotor leftEncoder, rightEncoder, backEncoder;
    private Buttons buttons;

    public EncoderTest(Gamepad gamepad1, Gamepad gamepad2) {
        buttons = new Buttons(gamepad1, gamepad2);
    }

    public void runOpMode() {
        leftEncoder = hardwareMap.dcMotor.get("backLeftMotor");
        rightEncoder = hardwareMap.dcMotor.get("intakeMotor");
        backEncoder = hardwareMap.dcMotor.get("frontLeftMotor");

        double REncode = 0;
        double LEncode = 0;
        double BEncode = 0;

        while (opModeIsActive()) {
            if (buttons.handlerX.Pressed()) {
                leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rightEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                backEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

            REncode = rightEncoder.getCurrentPosition();
            LEncode = leftEncoder.getCurrentPosition();
            BEncode = backEncoder.getCurrentPosition();

            telemetry.addData("Right encoder:", REncode);
            telemetry.addData("Left Encoder:", LEncode);
            telemetry.addData("Back Encoder:", BEncode);
            telemetry.update();
        }
    }
}
