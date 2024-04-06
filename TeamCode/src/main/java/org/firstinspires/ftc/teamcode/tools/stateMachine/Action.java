package org.firstinspires.ftc.teamcode.tools.stateMachine;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.tools.Global;

public class Action {
        protected ActionFunction performAction; // A function that performs the action and returns true if the action is complete
        private final String msg;

        // Constructor that sets the action
        public Action(String msg, ActionFunction performAction) {
            this.performAction = performAction;
            this.msg = msg;
        }

        // Method to determine if the action is complete based on the BooleanSupplier
        public boolean evaluate() {
            if (msg != null){
                Global.telemetry.addLine(msg);
            }
            return performAction.evaluate();
        }

        @NonNull
        @Override
        public String toString() {
            return msg;
        }
    }