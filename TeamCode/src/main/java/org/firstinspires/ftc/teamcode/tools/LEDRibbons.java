package org.firstinspires.ftc.teamcode.tools;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LEDRibbons {
    RevBlinkinLedDriver blinkinLedDriver1;
    RevBlinkinLedDriver blinkinLedDriver2;

    public LEDRibbons(HardwareMap hardwareMap) {
        blinkinLedDriver1 = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin1");
        blinkinLedDriver2 = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin2");
    }

    public void setPattern(RevBlinkinLedDriver.BlinkinPattern pattern1, RevBlinkinLedDriver.BlinkinPattern pattern2) {
        blinkinLedDriver1.setPattern(pattern1);
        blinkinLedDriver2.setPattern(pattern2);
    }
}

