package org.firstinspires.ftc.teamcode.tools.StateMachine;

import org.firstinspires.ftc.teamcode.Claw;

public class ClawActionBuilder {

    ClawActionBuilder (Claw claw) { this.claw = claw; }

    Action setGripPosition(Claw.gripPositions gripPosition) {
        ActionFunction function = () -> {
            claw.setGripPosition(gripPosition);
            return true;
        };
        return new Action ("setGripPosition", function);
    }

    Action setPitchPosition(Claw.pitchPositions pitchPosition) {
        ActionFunction function = () -> {
            claw.setPitchPosition(pitchPosition);
            return true;
        };
        return new Action ("setPitchPosition", function);
    }

    Action waitForAnalogPitchSensorAtPosition () {
        ActionFunction function = () -> {
            claw.waitForAnalogPitchSensorAtPosition();
            return true;
        };
        return new Action("waitForAnalogPitchSensorAtPosition", function);
    }


    Action setYawPosition(Claw.yawPositions yawPosition) {
        ActionFunction function = () -> {
            claw.setYawPosition(yawPosition);
            return true;
        };
        return new Action ("setYawPosition", function);
    }

    Action waitForAnalogYawSensorAtPosition () {
        ActionFunction function = () -> {
            claw.waitForAnalogYawSensorAtPosition();
            return true;
        };
        return new Action("waitForAnalogYawSensorAtPosition", function);
    }


    private Claw claw;
}
