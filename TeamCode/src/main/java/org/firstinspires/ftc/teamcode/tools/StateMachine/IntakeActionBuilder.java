package org.firstinspires.ftc.teamcode.tools.StateMachine;

import org.firstinspires.ftc.teamcode.Intake;

import java.util.concurrent.TimeUnit;

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

    public Action setFlipperServoPosition(Intake.FlipperPosition flipperPosition){
        ActionFunction function = () -> {
            intake.setIntakeFlipperPosition(flipperPosition);
            return true;
        };
        return new Action ("setFlipperServoPosition", function);
    }

    public Action waitForTwoPixelsOrTimeout(long timeout, TimeUnit unit){
        ActionFunction function = () -> {
            intake.pixels.update();
            return intake.pixels.hasTwoPixels();
        };
        return new DeadlineAction.DeadlineActionImpl("waitUntilHasTwoPixels", function, timeout, unit);
    }

    private final Intake intake;
}
