package org.firstinspires.ftc.teamcode.tools;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Global {
    public static Telemetry telemetry;

    public static int ensureWithin(int value, int valueMin, int valueMax) {
        return Math.min(Math.max(value, valueMin), valueMax);
    }
    public static double ensureWithin(double value, double valueMin, double valueMax) {
        return Math.min(Math.max(value, valueMin), valueMax);
    }
}
