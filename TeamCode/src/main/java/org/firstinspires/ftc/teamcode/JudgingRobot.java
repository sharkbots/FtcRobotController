package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.tools.stateMachine.StateMachine;

public class JudgingRobot extends Robot{
    private final StateMachine.State showAttachments;

    public JudgingRobot(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2) {
        super(hardwareMap, gamepad1, gamepad2);
        teleopDeadline.expire();

        showAttachments = new StateMachine.State("showAttachments");
        stateMachine.addState(showAttachments);

        //stateMachine.setInitialState(idle);


        idle.addTransitionTo(showAttachments, ()->true, outTake);
        //showAttachments.addTransitionTo(idle, buttons.handlerA::Pressed, resetOutTake);

        showAttachments.addTransitionTo(showAttachments, buttons.handlerLeftBumper::Pressed, closeClaw);
        showAttachments.addTransitionTo(showAttachments, buttons.handlerRightBumper::Pressed, openClaw);


    }
}
