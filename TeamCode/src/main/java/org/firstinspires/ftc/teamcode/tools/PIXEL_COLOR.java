package org.firstinspires.ftc.teamcode.tools;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

public enum PIXEL_COLOR {

    YELLOW(25.0, 125.0, RevBlinkinLedDriver.BlinkinPattern.ORANGE),
    GREEN(125.0, 160.0, RevBlinkinLedDriver.BlinkinPattern.DARK_GREEN),
    WHITE(160.0, 180.0, RevBlinkinLedDriver.BlinkinPattern.WHITE),
    PURPLE(180.0, 255.0, RevBlinkinLedDriver.BlinkinPattern.VIOLET),
    UNDEFINED(-1.0, -1.0, RevBlinkinLedDriver.BlinkinPattern.BLACK);

    private final double minHue;
    private final double maxHue;
    public final RevBlinkinLedDriver.BlinkinPattern pattern;

    static private final double MAX_DISTANCE_FROM_MAT_MM = 17.0;

    PIXEL_COLOR(double minHue, double maxHue, RevBlinkinLedDriver.BlinkinPattern pattern) {
        this.minHue = minHue;
        this.maxHue = maxHue;
        this.pattern = pattern;
    }

    public static PIXEL_COLOR detect(ColorSensorInfo sensorInfo) {
        if(sensorInfo.distanceMM() <= MAX_DISTANCE_FROM_MAT_MM) { // distance too far means we are seeing the mat


            for (PIXEL_COLOR color : PIXEL_COLOR.values()) {
                // range excludes maxHue value to not overlap with other color ranges
                if (sensorInfo.hue() >= color.minHue && sensorInfo.hue() < color.maxHue) {
                    // purple and white can be mismatched, but in case of true white the RBG values are all shooting up
                    if(color==PIXEL_COLOR.PURPLE && sensorInfo.red()>=190 && sensorInfo.green()>=190 && sensorInfo.blue()>=190) {
                        return PIXEL_COLOR.WHITE;
                    }
                    else {
                        return color;
                    }
                }
            }
        }
        return PIXEL_COLOR.UNDEFINED;
    }


}
