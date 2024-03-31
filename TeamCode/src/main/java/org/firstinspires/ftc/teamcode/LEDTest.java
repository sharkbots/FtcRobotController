package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.aprilTags.AprilTagDetection;
import org.firstinspires.ftc.teamcode.tools.AutoDataStorage;
import org.firstinspires.ftc.teamcode.tools.Global;
import org.firstinspires.ftc.teamcode.tools.LEDRibbons;
import org.firstinspires.ftc.teamcode.tools.PixelsDetection;
import org.firstinspires.ftc.teamcode.tools.SetDriveMotors;

import java.util.concurrent.TimeUnit;

@TeleOp(name = "A LED")
public class LEDTest extends LinearOpMode {
    private SetDriveMotors driveMotors;
//
//    private LEDRibbons ledRibbons;
//    private PixelsDetection pixelsDetection;


    AprilTagDetection aprilTagDetection;

    public void Setup(){
        Global.telemetry = telemetry;

        sleep(1000);

//        ledRibbons = new LEDRibbons(hardwareMap);
//        pixelsDetection = new PixelsDetection(hardwareMap);


        while(!isStarted() && !isStopRequested()){
        }
        AutoDataStorage.comingFromAutonomous = false;

    }

    @Override
    public void runOpMode() throws InterruptedException {
        Setup();
        waitForStart();

        DcMotor blinkinPower = hardwareMap.get(DcMotor.class, "blinkinpower");
        blinkinPower.setPower(1);

        Deadline deadline = new Deadline(300, TimeUnit.MILLISECONDS);
        deadline.reset();
        boolean displayPixel1 = true;

        RevBlinkinLedDriver blinkinLedDriver1 = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin1");
        telemetry.addLine(blinkinLedDriver1.getConnectionInfo());

        while(opModeIsActive()){

            blinkinLedDriver1.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
        }

    }

}