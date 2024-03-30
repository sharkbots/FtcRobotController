package org.firstinspires.ftc.teamcode.tools.StateMachine;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;


public class DeadlineAction {


    /*public Action resetTimer() {
        ActionFunction function = () -> { timer.resetTimer(); return true;};
        return new Action("reset timer ", function);
    }*/

    public static Action waitForMS(long durationInMS) {
        return waitFor(durationInMS, TimeUnit.MILLISECONDS);
    }

    public static Action waitFor(long duration, TimeUnit timeUnit) {
        String name = "wait for timer to exceed " + duration + " " + timeUnit;
        return new DeadlineActionImpl(name, duration, timeUnit);
    }

    public static class DeadlineActionImpl extends Action {
        private final Deadline deadline;
        private boolean isTimeoutStarted = false;

        public DeadlineActionImpl(String msg, long timeout, TimeUnit unit) {
            this(msg, () -> false, timeout, unit);
        }

        public DeadlineActionImpl(String msg, ActionFunction performActionButNoLongerThanTimeout, long timeout, TimeUnit unit) {
            super(msg, performActionButNoLongerThanTimeout);
            deadline = new Deadline(timeout, unit);
            super.performAction = () -> {  // Perform action and return true when action returns true, or when timeout is reached
                if(!isTimeoutStarted){
                    deadline.reset();
                    isTimeoutStarted = true;
                }
                if(deadline.hasExpired()) {
                    return true; // we timed out
                }
                return performActionButNoLongerThanTimeout.evaluate();
            };

        }
    }
}
