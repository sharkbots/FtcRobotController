package org.firstinspires.ftc.teamcode.tools;

import java.util.ArrayList;
import java.util.function.BooleanSupplier;

public class StateMachine {

    // Manages your current state
    public class State {
        public State(String name){
            this.name = name;
        }

        public void addTransition(Transition transition)
        {
            transitions.add(transition);
        }

        // Manages the transition from one state to another
        public class Transition
        {
            private BooleanSupplier trigger;
            private State nextState;

            // The current action in your transition
            public class Action
            {
                private BooleanSupplier performAction;
                public boolean actionComplete(){
                    return performAction.getAsBoolean();
                }

            }
            ArrayList<Action> actions = new ArrayList<>();
            int currentActionIndex;

            public boolean triggered(){
                return trigger.getAsBoolean();
            }
            public boolean transitionComplete(){
                //add return
                for (currentActionIndex = 0; currentActionIndex < actions.size(); currentActionIndex++)
                {
                    if (!(actions.get(currentActionIndex).actionComplete())) { return false;}
                }
                currentActionIndex = 0;
                return true;
            }
            public State getDestinationState(){
                return nextState;
            }
        }
        private ArrayList<Transition> transitions = new ArrayList<>();
        private Transition currentTransition = null;
        public String name = new String();
        State update() {
            // If no transition to new state, return this; otherwise return new state.
            if (currentTransition == null){
                for (Transition t : transitions){
                    if (t.triggered()){
                        currentTransition = t;
                        break;
                    }
                }
                return this;
            }
            if (currentTransition.transitionComplete()){
                State destinationState = currentTransition.getDestinationState();
                currentTransition = null;
                return destinationState;
            }
            return this;

        }

    }

    private ArrayList<State> states = new ArrayList<>();
    private State currentState = null;
    public void addState(State state) {
        states.add(state);
    }
    public void setInitialState(State state){
        assert currentState == null :"cannot set initial state twice";
        currentState = state;
    }

    public void updateState() {
        assert currentState != null :"initial state undefined";
        currentState = currentState.update();
    }
}