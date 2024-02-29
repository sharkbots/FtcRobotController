package org.firstinspires.ftc.teamcode.tools.StateMachine;

import java.util.ArrayList;

public class Actions {
    int currentActionIndex; // Index to keep track of the current action
    //ArrayList<Action> actions; // List of actions to be performed during the transition
    ArrayList<Action> actions;

    public Actions(ActionBuilder builder){
        this.actions = builder.getList();
        currentActionIndex = 0;
    }

    public void run(){
        while (!isComplete()){
            // do nothing
        }
    }

    public void runAsync(){
        class MyRunnable implements Runnable {
            @Override
            public void run() {
                Actions.this.run();
            }
        }
        Thread thread = new Thread(new MyRunnable());
        thread.start();
    }

    public boolean isComplete() {
        // Iterate through all actions to see if they are complete
        for (; currentActionIndex < actions.size(); currentActionIndex++) {
            if (!(actions.get(currentActionIndex).evaluate())) {
                return false;
            }
        }
        currentActionIndex = 0; // Reset the action index
        return true; // All actions are complete
    }

    public void update(){
        isComplete();
    }
}
