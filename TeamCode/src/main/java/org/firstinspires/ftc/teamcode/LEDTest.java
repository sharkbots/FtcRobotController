package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.aprilTags.AprilTagDetection;
import org.firstinspires.ftc.teamcode.tools.AutoDataStorage;
import org.firstinspires.ftc.teamcode.tools.Global;
import org.firstinspires.ftc.teamcode.tools.PixelsAsk;
import org.firstinspires.ftc.teamcode.tools.PixelsDetection;
import org.firstinspires.ftc.teamcode.tools.SetDriveMotors;

import java.util.concurrent.TimeUnit;

@TeleOp(name = "A LED")
public class LEDTest extends LinearOpMode {
    private SetDriveMotors driveMotors;

    private PixelsAsk pixelsToAsk;
    private PixelsDetection pixelsDetection;


    AprilTagDetection aprilTagDetection;

    public void Setup(){
        Global.telemetry = telemetry;

        sleep(1000);

        pixelsToAsk = new PixelsAsk(hardwareMap);
        pixelsDetection = new PixelsDetection(hardwareMap);


        while(!isStarted() && !isStopRequested()){
        }
        AutoDataStorage.comingFromAutonomous = false;

    }

    @Override
    public void runOpMode() throws InterruptedException {
        Setup();
        waitForStart();


        Deadline deadline = new Deadline(300, TimeUnit.MILLISECONDS);
        deadline.reset();
        boolean displayPixel1 = true;


        while(opModeIsActive()){

            pixelsDetection.update();
            if(deadline.hasExpired()) {
                displayPixel1 = !displayPixel1;
                deadline.reset();
            }
            pixelsToAsk.setPattern((displayPixel1?pixelsDetection.pixel1:pixelsDetection.pixel2).pattern);
        }


//            pattern = RevBlinkinLedDriver.BlinkinPattern.VIOLET;
//            blinkinLedDriver.setPattern(pattern);
//
//            pixelsLoaded.Zero();
//            sleep(500);
//            pixelsLoaded.One();
//            sleep(500);
//            pixelsLoaded.Two();
//            sleep(500);
//
//            for(int i = 0;i < 6; i++) {
//                //blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
//                //sleep(400);
//                pixelsToAsk.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE_VIOLET);
//                sleep(300);
//                pixelsToAsk.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
//                sleep(300);
//            }
//
//
//            for(int i = 0;i < 6; i++) {
//                //blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
//                //sleep(400);
//                pixelsToAsk.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE_VIOLET);
//                sleep(300);
//                pixelsToAsk.setPattern(RevBlinkinLedDriver.BlinkinPattern.DARK_GREEN);
//                sleep(300);
//            }


//        }
    }

}