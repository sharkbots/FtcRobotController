package org.firstinspires.ftc.teamcode.tools;

import com.qualcomm.robotcore.util.ElapsedTime;

public class Timer {
    private final ElapsedTime timer;
    public Timer(ElapsedTime timer){
        this.timer = timer;
    }
    public void resetTimer(){
        timer.reset();
    }

    public double getMilliseconds(){
        return timer.milliseconds();
    }
    public boolean waitUntil(double targetTime){
        return timer.milliseconds() >= targetTime;
    }
}