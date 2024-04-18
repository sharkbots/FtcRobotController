package org.firstinspires.ftc.teamcode.tools;

import static org.firstinspires.ftc.teamcode.tools.PIXEL_COLOR.MAX_DISTANCE_FROM_MAT_MM;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LED;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

public class PixelsDetection {

    public enum LEDMode {
        SOLID,
        BLINKING
    };

    private enum LEDColor {
        RED,
        GREEN,
        BLACK
    };


    private final RevColorSensorV3 colorSensor1;
    private final RevColorSensorV3 colorSensor2;

    public PIXEL_COLOR pixel1 = PIXEL_COLOR.UNDEFINED;
    public PIXEL_COLOR pixel2 = PIXEL_COLOR.UNDEFINED;


    private final ArrayList<LED> ledsRed = new ArrayList<LED>(4);
    private final ArrayList<LED> ledsGreen = new ArrayList<LED>(4);
    private LEDMode ledMode = LEDMode.SOLID;
    private final Deadline twoPixelsDetectionDelay, blinkingDelay;
    private boolean isLedOnForBlinking = true;



    public PixelsDetection(HardwareMap hardwareMap) {
        for(int i=0; i<4;i++) {
            ledsRed.add(  i, hardwareMap.get(LED.class, "led" + (i+1) + "red"));
            ledsGreen.add(i, hardwareMap.get(LED.class, "led" + (i+1) + "green"));
        }

        colorSensor1 = hardwareMap.get(RevColorSensorV3.class, "colorsensor1");
        colorSensor2 = hardwareMap.get(RevColorSensorV3.class, "colorsensor2");

        twoPixelsDetectionDelay = new Deadline(20, TimeUnit.MILLISECONDS);
        blinkingDelay = new Deadline(50, TimeUnit.MILLISECONDS);

        for(LED ledRed: ledsRed) {
            ledRed.enable(true);
            ledRed.enableLight(false);
        }
        for(LED ledGreen: ledsGreen) {
            ledGreen.enable(true);
            ledGreen.enableLight(false);
        }

        colorSensor1.enableLed(true);
        colorSensor2.enableLed(true);
        colorSensor1.setGain(5);
        colorSensor2.setGain(5);
    }


    public void update() {
        pixel1 = detectColorFrom(colorSensor1, "1- ");
        pixel2 = detectColorFrom(colorSensor2, "2- ");
        if(!hasTwoPixelsRaw()) {
            twoPixelsDetectionDelay.reset();
        }
        if(ledMode==LEDMode.BLINKING && blinkingDelay.hasExpired()){
            blinkingDelay.reset();
            isLedOnForBlinking = !isLedOnForBlinking;
        }

        setLEDStatus();
        Global.telemetry.update();
    }

    private PIXEL_COLOR detectColorFrom(RevColorSensorV3 colorSensor, String debugPrefix) {
        ColorSensorInfo colorSensorInfo = new ColorSensorInfo(colorSensor);

        PIXEL_COLOR detectedColor = PIXEL_COLOR.detect(colorSensorInfo,
                                                       debugPrefix=="1- "? PIXEL_COLOR.MAX_DISTANCE_FROM_MAT_MM: PIXEL_COLOR.MAX_DISTANCE_PIXEL2_FROM_MAT_MM);

        Global.telemetry.addLine()
                .addData(debugPrefix+"PIXEL",  detectedColor.name());


//        Global.telemetry.addLine()
//                .addData(debugPrefix+"R",  colorSensorInfo.red())
//                .addData(debugPrefix+"G", colorSensorInfo.green())
//                .addData(debugPrefix+"B",  colorSensorInfo.blue());
//        Global.telemetry.addLine()
//                .addData(debugPrefix+"Hue", "%.3f", colorSensorInfo.hue())
//                .addData(debugPrefix+"Sat", "%.3f", colorSensorInfo.saturation())
//                .addData(debugPrefix+"Val", "%.3f", colorSensorInfo.value());

        Global.telemetry.addData(debugPrefix+"Distance (mm)", "%.3f", colorSensorInfo.distanceMM());
        Global.telemetry.addLine();
        Global.telemetry.addLine();

        return detectedColor;
    }

    public void setLEDMode(LEDMode mode) {
        ledMode = mode;
    }
    public boolean hasTwoPixels() {
        // Add delay before acknowledging two pixels are in
        // Gives time to the intake getting the pixels really in
        // or avoid false positive signal on a passing first pixel through
        return  twoPixelsDetectionDelay.hasExpired() && hasTwoPixelsRaw();
    }

    public boolean hasTwoPixelsRaw() {
        return (pixel1 != PIXEL_COLOR.UNDEFINED) && (pixel2 != PIXEL_COLOR.UNDEFINED);
    }

    public boolean hasOnePixel() {
        return (pixel1 != PIXEL_COLOR.UNDEFINED) || (pixel2 != PIXEL_COLOR.UNDEFINED);
    }

    private void setLEDStatus() {
        if(ledMode==LEDMode.BLINKING && !isLedOnForBlinking) {
            setLED1(LEDColor.BLACK);
            setLED2(LEDColor.BLACK);
        } else if(hasTwoPixels()) { // For two pixels, comply to delay to provide visual feedback
            setLED1(LEDColor.GREEN);
            setLED2(LEDColor.GREEN);
        } else {
            setLED1((pixel1 == PIXEL_COLOR.UNDEFINED)?LEDColor.RED:LEDColor.GREEN);
            setLED2((pixel2 == PIXEL_COLOR.UNDEFINED)?LEDColor.RED:LEDColor.GREEN);
        }
    }
    private void setLED1(LEDColor ledColor) {
        setLED(0, ledColor);
    }
    private void setLED2(LEDColor ledColor) {
        setLED(1, ledColor);
    }
    private void setLED(int ledLogicalIndex, LEDColor ledColor) {
        ledLogicalIndex *= 2; // LEDs come in pairs
        ledsRed.get(ledLogicalIndex).enableLight(ledColor==LEDColor.RED);
        ledsRed.get(ledLogicalIndex+1).enableLight(ledColor==LEDColor.RED);
        ledsGreen.get(ledLogicalIndex).enableLight(ledColor==LEDColor.GREEN);
        ledsGreen.get(ledLogicalIndex+1).enableLight(ledColor==LEDColor.GREEN);
    }


}

