package org.firstinspires.ftc.teamcode.tools.StateMachine;

import org.firstinspires.ftc.teamcode.Claw;

public class ClawActionBuilder {

    public ClawActionBuilder (Claw claw) {this.claw = claw;}

    public Action setGripPosition(Claw.gripPositions gripPosition) {
        ActionFunction function = () -> {
            claw.setGripPosition(gripPosition);
            return true;
        };
        return new Action ("setGripPosition", function);
    }

    public Action setPitchPosition(Claw.pitchPositions pitchPosition) {
        ActionFunction function = () -> {
            claw.setPitchPosition(pitchPosition);
            return true;
        };
        return new Action ("setPitchPosition", function);
    }

    public Action waitForAnalogPitchSensorAtPosition (Claw.pitchPositions pitchPositions, double tolerance) {
        ActionFunction function = () -> {
            claw.waitForAnalogPitchSensorAtPosition(pitchPositions, tolerance);
            return true;
        };
        return new Action("waitForAnalogPitchSensorAtPosition", function);
    }


    public Action setYawPosition(Claw.yawPositions yawPosition) {
        ActionFunction function = () -> {
            claw.setYawPosition(yawPosition);
            return true;
        };
        return new Action ("setYawPosition", function);
    }

    public Action waitForAnalogYawSensorAtPosition (Claw.yawPositions yawPositions, double tolerance) {
        ActionFunction function = () -> claw.waitForAnalogYawSensorAtPosition(yawPositions, tolerance);
        return new Action("waitForAnalogYawSensorAtPosition", function);
    }


    private final Claw claw;
}
