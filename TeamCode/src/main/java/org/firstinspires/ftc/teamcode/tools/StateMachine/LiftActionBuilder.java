package org.firstinspires.ftc.teamcode.tools.StateMachine;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Lift;

public class LiftActionBuilder {
    LiftActionBuilder (Lift lift) {
        this.lift = lift;
    }

    Action setMinLiftPosition(Lift.liftPositions liftPosition) {
        ActionFunction function = () -> {
            lift.setLiftPosition(liftPosition);
            return true;
        };
        return new Action("setMinLiftPosition", function);
    }

    Action waitLiftMotorAbovePosition(Lift.liftPositions expectedPosition) {
        ActionFunction function = () -> {
            lift.waitLiftMotorAbovePosition(expectedPosition);
            return true;
        };
        return new Action("waitForLiftMotorAbovePosition", function);
    }

    Action waitLiftMotorBelowPosition(int expectedPosition) {
        ActionFunction function = () -> {
            lift.waitLiftMotorBelowPosition(expectedPosition);
            return true;
        };
        return new Action("waitLiftMotorBelowPosition", function);
    }

    Action resetLiftMotorEncoder() {
        ActionFunction function = () -> {
            lift.resetLiftMotorEncoder();
            return true;
        };
        return new Action("resetLiftMotorEncoder", function);
    }

    Action startMotorEncoder() {
        ActionFunction function = () -> {
            lift.startMotorEncoder();
            return true;
        };
        return new Action("startMotorEncoder", function);
    }

    Action startMotorNoEncoder() {
        ActionFunction function = () -> {
            lift.startMotorNoEncoder();
            return true;
        };
        return new Action("startMotorNoEncoder", function);
    }

    Action stopLiftMotor() {
        ActionFunction function = () -> {
            lift.stopLiftMotor();
            return true;
        };
        return new Action("stopLiftMotor", function);
    }

    private Lift lift;
}
