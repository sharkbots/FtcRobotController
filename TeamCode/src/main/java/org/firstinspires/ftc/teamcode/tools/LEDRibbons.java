package org.firstinspires.ftc.teamcode.tools;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

public class LEDRibbons {
    RevBlinkinLedDriver blinkinLedDriver1;
    //RevBlinkinLedDriver blinkinLedDriver2;

    private final Deadline deadline = new Deadline(150, TimeUnit.MILLISECONDS);
    private boolean displayPixel1 = true;
    private RevBlinkinLedDriver.BlinkinPattern pattern1 = RevBlinkinLedDriver.BlinkinPattern.DARK_BLUE;
    private RevBlinkinLedDriver.BlinkinPattern pattern2 = RevBlinkinLedDriver.BlinkinPattern.DARK_RED;;

    public LEDRibbons(HardwareMap hardwareMap) {
        blinkinLedDriver1 = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin1");
       // blinkinLedDriver2 = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin2");
        deadline.reset();
    }

    public void setPattern(RevBlinkinLedDriver.BlinkinPattern pattern1, RevBlinkinLedDriver.BlinkinPattern pattern2) {
        this.pattern1 = pattern1;
        this.pattern2 = pattern2;
    }

    public void update() {
            if(deadline.hasExpired()) {
                displayPixel1 = !displayPixel1;
                deadline.reset();
            }
             blinkinLedDriver1.setPattern(displayPixel1? pattern1:pattern2);

    }
}

