package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.tools.Global;
import org.firstinspires.ftc.teamcode.tools.SetDriveMotorsBlocking;

@TeleOp
public class fieldCentricBlocking extends LinearOpMode {
    private SetDriveMotorsBlocking driveMotors;
    public void Setup(){
        Global.telemetry = telemetry;
        driveMotors = new SetDriveMotorsBlocking(hardwareMap, gamepad1);


    }

    @Override
    public void runOpMode() throws InterruptedException {
        Setup();
        waitForStart();

        while (opModeIsActive()) {
            Global.telemetry.update();
            double horizontal = gamepad1.left_stick_x;
            double vertical = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            boolean goFast = gamepad1.left_bumper;

            boolean switchDriveMode = gamepad1.b;

            driveMotors.driveCommands(horizontal, vertical, turn, goFast, switchDriveMode);
        }
    }
}