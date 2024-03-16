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

    private Lift lift;
}
