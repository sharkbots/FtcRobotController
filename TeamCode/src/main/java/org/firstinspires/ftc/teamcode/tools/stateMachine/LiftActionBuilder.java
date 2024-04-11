package org.firstinspires.ftc.teamcode.tools.stateMachine;

import org.firstinspires.ftc.teamcode.Lift;

public class LiftActionBuilder {
    public LiftActionBuilder (Lift lift) {
        this.lift = lift;
    }

    public Action setLiftMotorPositionWithPower(Lift.Position position, double power) {  // done --> lift
        ActionFunction function = () -> {
            lift.setLiftPosition(position, power);
            return true;
        };
        return new Action("set lift motor to target position" , function);
    }

    public Action waitForLiftMotorAbovePosition(Lift.Position expectedPosition) {
        ActionFunction function = () -> lift.waitLiftMotorAbovePosition(expectedPosition);
        return new Action("waitForLiftMotorAbovePosition", function);
    }

    public Action waitForLiftMotorBelowPosition(int expectedPosition) {
        ActionFunction function = () -> lift.waitLiftMotorBelowPosition(expectedPosition);
        return new Action("waitLiftMotorBelowPosition", function);
    }

    public Action resetLiftMotorEncoder() {
        ActionFunction function = () -> {
            lift.resetLiftMotorEncoder();
            return true;
        };
        return new Action("resetLiftMotorEncoder", function);
    }

    public Action startMotorWithEncoder(double power) {
        ActionFunction function = () -> {
            lift.startLiftMotorWithEncoder(power);
            return true;
        };
        return new Action("startMotorEncoder", function);
    }

    public Action startMotorNoEncoder(double power) {
        ActionFunction function = () -> {
            lift.startLiftMotorWithNoEncoder(power);
            return true;
        };
        return new Action("startMotorNoEncoder", function);
    }

    public Action stopLiftMotor() {
        ActionFunction function = () -> {
            lift.stopLiftMotor();
            return true;
        };
        return new Action("stopLiftMotor", function);
    }

    public Action waitUntilLiftTouchDownPressed(){
        ActionFunction function = lift::liftTouchDownPressed;
        return new Action("waitUntilLiftTouchDownPressed", function);
    }

    private final Lift lift;
}
