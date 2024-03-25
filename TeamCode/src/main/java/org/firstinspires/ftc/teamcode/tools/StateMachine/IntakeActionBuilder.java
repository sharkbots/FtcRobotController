package org.firstinspires.ftc.teamcode.tools.StateMachine;

import org.firstinspires.ftc.teamcode.Intake;

public class IntakeActionBuilder {
    public IntakeActionBuilder (Intake intake) {
        this.intake = intake;
    }

    public Action startIntakeMotorWithNoEncoder(double power) {
        ActionFunction function = () -> {
            intake.startIntakeMotorWithNoEncoder(power);
            return true;
        };
        return new Action("startMotorNoEncoder", function);
    }

    public Action stopIntakeMotor() {
        ActionFunction function = () -> {
            intake.stopIntakeMotor();
            return true;
        };
        return new Action("stopLiftMotor", function);
    }

    public Action setFlipperServoPosition(Intake.flipperPositions flipperPosition){
        ActionFunction function = () -> {
            intake.setIntakeFlipperPosition(flipperPosition);
            return true;
        };
        return new Action ("setFlipperServoPosition", function);
    }

    private final Intake intake;
}
