package org.firstinspires.ftc.teamcode.tools.StateMachine;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;


public class DeadlineAction extends Action {


    public static Action waitFor(long duration, TimeUnit timeUnit) {
        String name = "wait for timer to exceed " + duration + " " + timeUnit;
        return new DeadlineAction(name, duration, timeUnit);
    }

    private final Deadline deadline;
        private boolean isTimeoutStarted = false;

        public DeadlineAction(String msg, long timeout, TimeUnit unit) {
            this(msg, () -> false, timeout, unit);
        }

        public DeadlineAction(String msg, ActionFunction performActionButNoLongerThanTimeout, long timeout, TimeUnit unit) {
            super(msg, performActionButNoLongerThanTimeout);
            deadline = new Deadline(timeout, unit);
            super.performAction = () -> {  // Perform action and return true when action returns true, or when timeout is reached
                if(!isTimeoutStarted){
                    deadline.reset();
                    isTimeoutStarted = true;
                }
                if(deadline.hasExpired() || performActionButNoLongerThanTimeout.evaluate()) {
                    isTimeoutStarted = false; // reset deadline for next round
                    return true; // we timed out or action finished
                }
                return false;
            };
        }
    }

