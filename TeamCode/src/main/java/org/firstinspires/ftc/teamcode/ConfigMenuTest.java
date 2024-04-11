package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.autonomous.AutoBase;
import org.firstinspires.ftc.teamcode.tools.AutoDataStorage;
import org.firstinspires.ftc.teamcode.tools.Buttons;
import org.firstinspires.ftc.teamcode.tools.Global;
import org.firstinspires.ftc.teamcode.tools.PIXEL_COLOR;

@TeleOp(name = "A CONFIG MENU")
public class ConfigMenuTest extends LinearOpMode {

    //AutoBase.Coordinates c;
    Buttons buttons;
    ConfigMenu menu;

    class TEST {
        PIXEL_COLOR pixel = PIXEL_COLOR.UNDEFINED;
        boolean bool = false;
        int integer = 0;
        float fl = 0.0f;
        double dbl = 0.0;
    }
    public void Setup() {
        Global.telemetry = telemetry;
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
        sleep(500);

        //c = new AutoBase.Coordinates(true, true);
        buttons = new Buttons(gamepad1, gamepad2);
        menu = new ConfigMenu(new TEST(), buttons);

        while(!isStarted() && !isStopRequested()){
        }
        AutoDataStorage.comingFromAutonomous = false;

    }

    @Override
    public void runOpMode() {
        Setup();
        waitForStart();
        Global.telemetry.speak("SHARKBOTS SHARKBOTS SHARKBOTS");
        while(opModeIsActive()){
            telemetry.addLine("<!doctype html><html><head><title>Logs</title></head><body>" +
                    "<b>TEST </b><i>TEST</i><p style=\"color:yellow\">GREEN</p></body></html>");
            menu.update();
        }

    }
}