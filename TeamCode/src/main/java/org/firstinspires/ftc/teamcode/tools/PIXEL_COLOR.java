package org.firstinspires.ftc.teamcode.tools;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

public enum PIXEL_COLOR {

    YELLOW(65.0, 105.0, RevBlinkinLedDriver.BlinkinPattern.ORANGE),
    GREEN(105.0, 145.0, RevBlinkinLedDriver.BlinkinPattern.DARK_GREEN),
    WHITE(145.0, 185.0, RevBlinkinLedDriver.BlinkinPattern.WHITE),
    PURPLE(185.0, 225.0, RevBlinkinLedDriver.BlinkinPattern.VIOLET),
    UNDEFINED(-1.0, -1.0, RevBlinkinLedDriver.BlinkinPattern.BLACK);

    private final double minHue;
    private final double maxHue;
    public final RevBlinkinLedDriver.BlinkinPattern pattern;

    static private final double MAX_DISTANCE_FROM_MAT_MM = 20.0;

    PIXEL_COLOR(double minHue, double maxHue, RevBlinkinLedDriver.BlinkinPattern pattern) {
        this.minHue = minHue;
        this.maxHue = maxHue;
        this.pattern = pattern;
    }

    public static PIXEL_COLOR detect(ColorSensorInfo sensorInfo) {
        if(sensorInfo.distanceMM() <= MAX_DISTANCE_FROM_MAT_MM) { // distance too far means we are seeing the mat
            for (PIXEL_COLOR color : PIXEL_COLOR.values()) {
                if (sensorInfo.hue() >= color.minHue && sensorInfo.hue() < color.maxHue) { // range excludes max value to not overlap with other color ranges
                    return color;
                }
            }
        }
        return PIXEL_COLOR.UNDEFINED;
    }


}
