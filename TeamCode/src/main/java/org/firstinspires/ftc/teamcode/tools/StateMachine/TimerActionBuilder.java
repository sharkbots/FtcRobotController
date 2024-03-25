package org.firstinspires.ftc.teamcode.tools.StateMachine;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.tools.Timer;


public class TimerActionBuilder {

    public TimerActionBuilder (Timer timer) { this.timer = timer; }

    public Action resetTimer() {
        ActionFunction function = () -> { timer.resetTimer(); return true;};
        return new Action("reset timer ", function);
    }

    public Action waitUntil(double targetTime) {
        ActionFunction function = () -> timer.waitUntil(targetTime);
        String name = "wait for timer to exceed " + targetTime;
        return new Action(name, function);
    }


    private final Timer timer;
}
