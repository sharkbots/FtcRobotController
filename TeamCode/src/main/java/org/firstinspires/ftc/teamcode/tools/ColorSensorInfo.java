package org.firstinspires.ftc.teamcode.tools;

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class ColorSensorInfo {

    static private final float scale = 256.0F*20; // multiply by a gain as values are very small from Color Sensor v3
    static private final int min = 0;
    static private final int max = 255;

    private final NormalizedRGBA normRGBA;
    private final double distanceMM;
    private final float[] hsvValues = new float[3];

    ColorSensorInfo(RevColorSensorV3 colorSensor) {
        this.normRGBA = colorSensor.getNormalizedColors();
        this.distanceMM = colorSensor.getDistance(DistanceUnit.MM);
        Color.colorToHSV(this.toColor(), hsvValues);

    }

    int toColor()   {return normRGBA.toColor();}

    double distanceMM() {return distanceMM;}

    int alpha() {return Range.clip((int)(normRGBA.alpha * scale), min, max);}
    int red()   {return Range.clip((int)(normRGBA.red * scale), min, max);}
    int green() {return Range.clip((int)(normRGBA.green * scale), min, max);}
    int blue()  {return Range.clip((int)(normRGBA.blue * scale), min, max);}
    double hue()        {return hsvValues[0];}
    double saturation() {return hsvValues[1];}
    double value()      {return hsvValues[2];}
}
