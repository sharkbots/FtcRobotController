package org.firstinspires.ftc.teamcode.tools.StateMachine;

import java.util.ArrayList;

public class Actions {
    private int currentActionIndex; // Index to keep track of the current action
    private ArrayList<Action> actionList;  // List of actions to be performed during the transition

    public Actions(Action... action){
        actionList = new ArrayList<Action>();
        currentActionIndex = 0;
        add(action);
    }

    // Add a single action to the action list
    Actions add (Action... action) {
        for (Action a : action)
            actionList.add(a);
        return this;
    }

    // Add every action in a list of actions to overall actionList
    Actions add (Actions actions) {
        for (Action a : actions.actionList)
            actionList.add(a);
        return this;
    }

    boolean isComplete() {
        assert (currentActionIndex >= 0);
        while (true) {
            // if index is last, reached end of list
            if (currentActionIndex == actionList.size()-1) {
                currentActionIndex = 0;
                return true;
            }
            if (!actionList.get(currentActionIndex).evaluate()) {
                // Failed an execute, report failure.
                return false;
            }
            currentActionIndex ++;
        }
    }

    public void update(){
        isComplete();
    }
}
