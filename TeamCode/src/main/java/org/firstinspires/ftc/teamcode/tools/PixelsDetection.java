package org.firstinspires.ftc.teamcode.tools;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LED;

public class PixelsDetection {

//    private final LED led1red;
//    private final LED led1green;
//    private final LED led2red;
//    private final LED led2green;
    private final RevColorSensorV3 colorSensor1;
    private final RevColorSensorV3 colorSensor2;

    public PIXEL_COLOR pixel1 = PIXEL_COLOR.UNDEFINED;
    public PIXEL_COLOR pixel2 = PIXEL_COLOR.UNDEFINED;


    public PixelsDetection(HardwareMap hardwareMap) {
//        led1red = hardwareMap.get(LED.class, "led1red");
//        led1green = hardwareMap.get(LED.class, "led1green");
//        led2red = hardwareMap.get(LED.class, "led2red");
//        led2green = hardwareMap.get(LED.class, "led2green");

        colorSensor1 = hardwareMap.get(RevColorSensorV3.class, "colorsensor1");
        colorSensor2 = hardwareMap.get(RevColorSensorV3.class, "colorsensor2");

//        led1red.enable(true);
//        led1green.enable(true);
//        led2red.enable(true);
//        led2green.enable(true);
//
//        led1red.enableLight(false);
//        led1green.enableLight(false);
//        led2red.enableLight(false);
//        led2green.enableLight(false);

        colorSensor1.enableLed(true);
        colorSensor2.enableLed(true);
    }


    public void update() {
        pixel1 = detectColorFrom(colorSensor1, "1- ");
        pixel2 = detectColorFrom(colorSensor2, "2- ");
        setLEDStatus();
        Global.telemetry.update();


    }

    private PIXEL_COLOR detectColorFrom(RevColorSensorV3 colorSensor, String debugPrefix) {
        ColorSensorInfo colorSensorInfo = new ColorSensorInfo(colorSensor);

        PIXEL_COLOR detectedColor = PIXEL_COLOR.detect(colorSensorInfo);

        Global.telemetry.addLine()
                .addData(debugPrefix+"PIXEL",  detectedColor.name());


        Global.telemetry.addLine()
                .addData(debugPrefix+"R",  colorSensorInfo.red())
                .addData(debugPrefix+"G", colorSensorInfo.green())
                .addData(debugPrefix+"B",  colorSensorInfo.blue());
        Global.telemetry.addLine()
                .addData(debugPrefix+"Hue", "%.3f", colorSensorInfo.hue())
                .addData(debugPrefix+"Sat", "%.3f", colorSensorInfo.saturation())
                .addData(debugPrefix+"Val", "%.3f", colorSensorInfo.value());

        Global.telemetry.addData(debugPrefix+"Distance (mm)", "%.3f", colorSensorInfo.distanceMM());
        Global.telemetry.addLine();
        Global.telemetry.addLine();

        return detectedColor;
    }

    private void setLEDStatus() {
//        led1red.enableLight(pixel1==PIXEL_COLOR.UNDEFINED);
//        led1green.enableLight(pixel1!=PIXEL_COLOR.UNDEFINED);
//        led2red.enableLight(pixel2==PIXEL_COLOR.UNDEFINED);
//        led2green.enableLight(pixel2!=PIXEL_COLOR.UNDEFINED);
    }



}

