package org.firstinspires.ftc.teamcode.tools.stateMachine;

import androidx.annotation.NonNull;

import java.util.ArrayList;
import java.util.Arrays;

public class Actions {
    private int currentActionIndex; // Index to keep track of the current action
    private ArrayList<Action> actionList;  // List of actions to be performed during the transition

    public Actions(Action... action){
        actionList = new ArrayList<Action>();
        currentActionIndex = 0;
        add(action);
    }

    // Add a single action to the action list
    public Actions add (Action... action) {
        actionList.addAll(Arrays.asList(action));
        return this;
    }

    // Add every action in a list of actions to overall actionList
    public Actions add (Actions actions) {
        actionList.addAll(actions.actionList);
        return this;
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

    public Runnable getAsyncRunnable(){
        class MyAsyncRunnable implements Runnable {
            @Override
            public void run() {
                Actions.this.runAsync();
            }
        }
        return new MyAsyncRunnable();
    }

    public Runnable getRunnable(){
        class MyAsyncRunnable implements Runnable {
            @Override
            public void run() {
                Actions.this.run();
            }
        }
        return new MyAsyncRunnable();
    }

    public boolean isComplete() {
        // Iterate through all actions to see if they are complete
        for (; currentActionIndex < actionList.size(); currentActionIndex++) {
            if (!(actionList.get(currentActionIndex).evaluate())) {
                return false;
            }
        }
        currentActionIndex = 0; // Reset the action index
        return true; // All actions are complete
    }

    public void update(){
        isComplete();
    }

    @NonNull
    @Override
    public String toString() {
        StringBuilder sb = new StringBuilder();
        for (Action action: actionList) {
            // Assuming each Action has a meaningful toString() implementation
            sb.append(action.toString());
            sb.append(", "); // Separate actions with commas
            }
        return sb.toString();
    }
}