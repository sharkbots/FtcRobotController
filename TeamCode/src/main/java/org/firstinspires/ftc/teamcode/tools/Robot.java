package org.firstinspires.ftc.teamcode.tools;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Claw;
import org.firstinspires.ftc.teamcode.Hanger;
import org.firstinspires.ftc.teamcode.Intake;
import org.firstinspires.ftc.teamcode.Lift;
import org.firstinspires.ftc.teamcode.PlaneLauncher;
import org.firstinspires.ftc.teamcode.tools.StateMachine.Actions;
import org.firstinspires.ftc.teamcode.tools.StateMachine.ClawActionBuilder;
import org.firstinspires.ftc.teamcode.tools.StateMachine.IntakeActionBuilder;
import org.firstinspires.ftc.teamcode.tools.StateMachine.LiftActionBuilder;
import org.firstinspires.ftc.teamcode.tools.StateMachine.StateMachine;
import org.firstinspires.ftc.teamcode.tools.StateMachine.TimerActionBuilder;

import java.util.function.BooleanSupplier;

public class Robot {
    public Robot(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2, Boolean isAutonomousMode) {

        // Gamepads
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;

        // Button assignment
        handlerA = new Button(this.gamepad2, Button.NAME.A);
        handlerB = new Button(this.gamepad2, Button.NAME.B);
        handlerX = new Button(this.gamepad2, Button.NAME.X);
        handlerY = new Button(this.gamepad2, Button.NAME.Y);
        handlerLeftBumper = new Button(this.gamepad2, Button.NAME.LEFT_BUMPER);
        handlerRightBumper = new Button(this.gamepad2, Button.NAME.RIGHT_BUMPER);
        handlerLeftTrigger = new Button(this.gamepad2, Button.NAME.LEFT_TRIGGER);
        handlerRightTrigger = new Button(this.gamepad2, Button.NAME.RIGHT_TRIGGER);
        handlerDPad_Down = new Button(this.gamepad2, Button.NAME.DPAD_DOWN);
        handlerDPad_Up = new Button(this.gamepad2, Button.NAME.DPAD_UP);
        handlerDPad_Left = new Button(this.gamepad2, Button.NAME.DPAD_LEFT);
        handlerDPad_Right = new Button(this.gamepad2, Button.NAME.DPAD_RIGHT);
        handlerLeftStick_Up = new Button(this.gamepad2, Button.NAME.LEFT_STICK_UP);
        handlerLeftStick_Down = new Button(this.gamepad2, Button.NAME.LEFT_STICK_DOWN);
        handlerLeftStick_Left = new Button(this.gamepad2, Button.NAME.LEFT_STICK_LEFT);
        handlerLeftStick_Right = new Button(this.gamepad2, Button.NAME.LEFT_STICK_RIGHT);


        lift = new Lift(hardwareMap, gamepad2);
        claw = new Claw(hardwareMap, gamepad2);
        planeLauncher = new PlaneLauncher(hardwareMap, handlerX);
        skyHook = new Hanger(hardwareMap, handlerDPad_Down, handlerDPad_Up, handlerY);//hardwareMap.dcMotor.get("skyHookMotor");
        intake = new Intake(hardwareMap, handlerLeftTrigger, handlerLeftStick_Up, handlerLeftStick_Down);

        // Timer
        teleopTimer = new ElapsedTime();
        timer = new Timer(new ElapsedTime());

        clawActionBuilder = new ClawActionBuilder(claw);
        liftActionBuilder = new LiftActionBuilder(lift);
        timerActionBuilder = new TimerActionBuilder(timer);
        intakeActionBuilder = new IntakeActionBuilder(intake);


        stateMachine = new StateMachine();

        // States
        intakingPixels = new StateMachine.State("pixelTransition");
        holdingPixels = new StateMachine.State("holdingPixels");
        idle = new StateMachine.State("idle");
        outTakingPixels = new StateMachine.State("outTakingPixels");
        exitingOutTake = new StateMachine.State("exitingOutTake");

        // Adding states to stateMachine
        stateMachine.addState(intakingPixels);
        stateMachine.addState(holdingPixels);
        stateMachine.addState(holdingPixelsLow);
        stateMachine.addState(idle);
        stateMachine.addState(outTakingPixels);
        stateMachine.addState(exitingOutTake);


        // Set initial state
        stateMachine.setInitialState(idle);

        // Actions
        empty = new Actions();
        //teleop
        idleToIntakingPixels = new Actions()
                .add(clawActionBuilder.setGripPosition(Claw.gripPositions.OPEN))
                .add(timerActionBuilder.resetTimer())
                .add(timerActionBuilder.waitUntil(150))
                .add(liftActionBuilder.startMotorWithEncoder(-1))
                .add(liftActionBuilder.waitUntilLiftTouchDownPressed())
                .add(liftActionBuilder.stopLiftMotor())
                .add(liftActionBuilder.resetLiftMotorEncoder())
                .add(clawActionBuilder.setPitchPosition(Claw.pitchPositions.INTAKE))
                .add(intakeActionBuilder.startIntakeMotorWithNoEncoder(1));

                /*.servoRunToPosition(clawGrip, clawOpen)
                .resetTimer(timer)
                .waitUntil(timer, 150)
                .startMotor(lift.liftMotor, -1, true)
                .waitForTouchSensorPressed(liftTouchDown)
                .stopMotor(lift.liftMotor)
                .resetMotorEncoder(lift.liftMotor)
                .servoRunToPosition(clawPitch, clawPitchIntake)
                .startMotor(intakeMotor, 1, false));*/

        intakingToHoldingPixels = new Actions()
                .add(clawActionBuilder.setGripPosition(Claw.gripPositions.CLOSE))
                .add(intakeActionBuilder.stopIntakeMotor())
                .add(clawActionBuilder.setPitchPosition(Claw.pitchPositions.INTAKE))
                .add(timerActionBuilder.resetTimer())
                .add(timerActionBuilder.waitUntil(150))
                .add(liftActionBuilder.setLiftMotorPositionWithPower(Lift.Position.HOLDING_TELEOP, 1));


                /*.servoRunToPosition(clawGrip, clawClose)
                .stopMotor(intakeMotor)
                .servoRunToPosition(clawPitch, Robot.clawPitchIntake)
                .resetTimer(timer)
                .waitUntil(timer, 150)
                .setMotorPosition(lift.liftMotor, lift.liftEncoderHoldingTeleop, 1));*/

        holdingPixelsToIntakingPixels = new Actions()
                .add(clawActionBuilder.setPitchPosition(Claw.pitchPositions.INTAKE))
                .add(clawActionBuilder.setGripPosition(Claw.gripPositions.OPEN))
                .add(liftActionBuilder.startMotorWithEncoder(-1))
                .add(liftActionBuilder.waitUntilLiftTouchDownPressed())
                .add(liftActionBuilder.stopLiftMotor())
                .add(liftActionBuilder.resetLiftMotorEncoder())
                .add(intakeActionBuilder.startIntakeMotorWithNoEncoder(1));

                /*.servoRunToPosition(clawPitch, clawPitchIntake)
                .servoRunToPosition(clawGrip, clawOpen)
                .startMotor(lift.liftMotor, -1, true)
                .waitForTouchSensorPressed(liftTouchDown)
                .stopMotor(lift.liftMotor)
                .resetMotorEncoder(lift.liftMotor)
                .startMotor(intakeMotor, 1, false));*/

        holdingPixelsToIdle = new Actions()
                .add(clawActionBuilder.setGripPosition(Claw.gripPositions.OPEN));
                /*.servoRunToPosition(clawGrip, clawOpen));*/

        idleToHoldingPixels = new Actions()
                .add(clawActionBuilder.setGripPosition(Claw.gripPositions.CLOSE))
                .add(timerActionBuilder.resetTimer())
                .add(timerActionBuilder.waitUntil(200))
                .add(liftActionBuilder.setLiftMotorPositionWithPower(Lift.Position.HOLDING_TELEOP, 1));

                /*.servoRunToPosition(clawGrip, clawClose)
                .resetTimer(timer)
                .waitUntil(timer, 200)
                .setMotorPosition(lift.liftMotor, lift.liftEncoderHoldingTeleop, 1));*/

        holdingPixelsToOutTakingPixels = new Actions()
                .add(liftActionBuilder.setLiftMotorPositionWithPower(Lift.Position.MIN, 1))
                .add(liftActionBuilder.waitForLiftMotorAbovePosition(Lift.Position.MIN))
                .add(clawActionBuilder.setPitchPosition(Claw.pitchPositions.OUTTAKE));

                /*.setMotorPosition(lift.liftMotor, lift.liftEncoderMin, 1)
                .waitForMotorAbovePosition(lift.liftMotor, lift.liftEncoderMin)
                .servoRunToPosition(clawPitch, clawPitchOutTake));*/

        exitingOutTakeToIdle = new Actions()
                .add(clawActionBuilder.setYawPosition(Claw.yawPositions.INTAKE))
                .add(clawActionBuilder.waitForAnalogYawSensorAtPosition(Claw.yawPositions.RESET,5))
                .add(liftActionBuilder.setLiftMotorPositionWithPower(Lift.Position.MIN, 1))
                .add(liftActionBuilder.waitForLiftMotorAbovePosition(Lift.Position.MIN))
                .add(clawActionBuilder.setPitchPosition(Claw.pitchPositions.INTAKE))
                // To get lift going down as fast as possible, bring it down with motor power instead of servo
                // servo will act as maintaining a linear speed and it's slower than just motor power with help of gravity
                .add(clawActionBuilder.setPitchPosition(Claw.pitchPositions.INTAKE))
                .add(clawActionBuilder.waitForAnalogPitchSensorAtPosition(Claw.pitchPositions.RESET, 10))
                .add(liftActionBuilder.startMotorNoEncoder(-1))
                .add(liftActionBuilder.waitUntilLiftTouchDownPressed())
                .add(liftActionBuilder.stopLiftMotor())
                .add(liftActionBuilder.resetLiftMotorEncoder());

                /*.servoRunToPosition(clawYaw, clawYawIntake)
                .waitForAnalogSensorAtPosition(clawYawAnalogSensor, analog_ClawYaw_ResetPosition, 5)
                .setMotorPosition(lift.liftMotor, lift.liftEncoderMin, 1)
                .waitForMotorAbovePosition(lift.liftMotor, lift.liftEncoderMin)
                .servoRunToPosition(clawPitch, clawPitchIntake)
                // To get lift going down as fast as possible, bring it down with motor power instead of servo
                // servo will act as maintaining a linear speed and it's slower than just motor power with help of gravity
                .servoRunToPosition(clawPitch, clawPitchIntake)
                .waitForAnalogSensorAtPosition(clawPitchAnalogSensor, analog_ClawPitch_ResetPosition, 10)
                .startMotor(lift.liftMotor, -1, false)
                .waitForTouchSensorPressed(liftTouchDown)
                .stopMotor(lift.liftMotor)
                .resetMotorEncoder(lift.liftMotor));*/


        //auto
        autoHoldOnePixel = new Actions()
                .add(clawActionBuilder.setGripPosition(Claw.gripPositions.CLOSE_ONE_PIXEL))
                .add(timerActionBuilder.waitUntil(500))
                .add(liftActionBuilder.setLiftMotorPositionWithPower(Lift.Position.HOLDING, 1));

                /*.servoRunToPosition(clawGrip, clawCloseOnePixel)
                .waitUntil(timer, 500)
                .setMotorPosition(lift.liftMotor, lift.liftEncoderHolding, 1));*/


        autoOutTakeYellow = new Actions()
                .add(liftActionBuilder.setLiftMotorPositionWithPower(Lift.Position.AUTO_MIN_YELLOW, 1))
                .add(liftActionBuilder.waitForLiftMotorAbovePosition(Lift.Position.AUTO_MIN_YELLOW))
                .add(clawActionBuilder.setPitchPosition(Claw.pitchPositions.OUTTAKE));

                /*.setMotorPosition(lift.liftMotor, lift.liftEncoderMin-100, 1)
                .waitForMotorAbovePosition(lift.liftMotor, lift.liftEncoderMin-100)
                .servoRunToPosition(clawPitch, clawPitchOutTake));*/

        autoOutTakeYellowHigh = new Actions()
                .add(liftActionBuilder.setLiftMotorPositionWithPower(Lift.Position.AUTO_MIN_YELLOW_HIGH, 1))
                .add(liftActionBuilder.waitForLiftMotorAbovePosition(Lift.Position.AUTO_MIN_YELLOW_HIGH))
                .add(clawActionBuilder.setPitchPosition(Claw.pitchPositions.OUTTAKE));

                /*.setMotorPosition(lift.liftMotor, lift.liftEncoderMin+300, 1)
                .waitForMotorAbovePosition(lift.liftMotor, lift.liftEncoderMin+300)
                .servoRunToPosition(clawPitch, clawPitchOutTake));*/

        autoOutTakeYellowLow = new Actions()
                .add(liftActionBuilder.setLiftMotorPositionWithPower(Lift.Position.AUTO_MIN_YELLOW_LOW, 1))
                .add(liftActionBuilder.waitForLiftMotorAbovePosition(Lift.Position.AUTO_MIN_YELLOW_LOW));

                /*.setMotorPosition(lift.liftMotor, lift.liftEncoderHoldingLow, 1)
                .waitForMotorAbovePosition(lift.liftMotor, lift.liftEncoderHoldingLow));*/

        autoOpenClaw = new Actions()
                .add(clawActionBuilder.setGripPosition(Claw.gripPositions.OPEN_AUTO))
                .add(timerActionBuilder.resetTimer())
                .add(timerActionBuilder.waitUntil(300))
                .add(clawActionBuilder.setGripPosition(Claw.gripPositions.OPEN))
                .add(timerActionBuilder.resetTimer())
                .add(timerActionBuilder.waitUntil(300))
                .add(clawActionBuilder.setPitchPosition(Claw.pitchPositions.INTAKE))
                .add(timerActionBuilder.resetTimer())
                .add(timerActionBuilder.waitUntil(150));

                /*.servoRunToPosition(clawGrip, clawOpen/2)
                .resetTimer(timer)
                .waitUntil(timer, 300)
                .servoRunToPosition(clawGrip, clawOpen)
                .resetTimer(timer)
                .waitUntil(timer, 300)
                .servoRunToPosition(clawPitch, clawPitchIntake)
                .resetTimer(timer)
                .waitUntil(timer, 150));*/


        if (!isAutonomousMode) { // no state machine to create in autonomous
            createTeleopStateTransitions();
        }

    }





    private void createTeleopStateTransitions() {
        // button triggers
        BooleanSupplier handlerButtonAPressed = handlerA::Pressed;
        BooleanSupplier handlerButtonBPressed = handlerB::Pressed;
        // BooleanSupplier handlerButtonXPressed = handlerX::Pressed;
        // BooleanSupplier handlerButtonYPressed = handlerY::Pressed;
        // BooleanSupplier handlerButtonLeftBumperPressed = handlerLeftBumper::Pressed;
        // BooleanSupplier handlerButtonRightBumperPressed = handlerRightBumper::Pressed;
        BooleanSupplier handlerButtonLeftTriggerPressed = handlerLeftTrigger::Pressed;
        // BooleanSupplier handlerButtonRightTriggerPressed = handlerRightTrigger::Pressed;
        // BooleanSupplier handlerDPad_DownPressed = handlerDPad_Down::Pressed;
        // BooleanSupplier handlerDPad_UpPressed = handlerDPad_Up::Pressed;
        // BooleanSupplier handlerDPad_LeftPressed = handlerDPad_Left::Pressed;
        // BooleanSupplier handlerDPad_RightPressed = handlerDPad_Right::Pressed;

        BooleanSupplier alwaysTrue = ()-> true;


        // adding transitions

        // intaking pixels
        idle.addTransitionTo(intakingPixels, handlerButtonAPressed,
                idleToIntakingPixels);

        intakingPixels.addTransitionTo(holdingPixels, handlerButtonAPressed,
                intakingToHoldingPixels);

        holdingPixels.addTransitionTo(intakingPixels, handlerButtonAPressed,
                holdingPixelsToIntakingPixels);

        // rejecting pixels
        holdingPixels.addTransitionTo(idle, handlerButtonLeftTriggerPressed,
                holdingPixelsToIdle);

        idle.addTransitionTo(holdingPixels, handlerButtonLeftTriggerPressed,
                idleToHoldingPixels);

        holdingPixels.addTransitionTo(outTakingPixels, handlerButtonBPressed,
                holdingPixelsToOutTakingPixels);

        // in exitingOutTake state, lift cannot be controlled manually
        outTakingPixels.addTransitionTo(exitingOutTake, handlerButtonBPressed,
                empty);

        exitingOutTake.addTransitionTo(idle, alwaysTrue,
                exitingOutTakeToIdle);
    }

    // Functionalities
    public static Lift lift;
    public static Claw claw;
    public static PlaneLauncher planeLauncher;
    public static Intake intake;

    // Action Builders
    public static ClawActionBuilder clawActionBuilder;
    public static LiftActionBuilder liftActionBuilder;
    public static TimerActionBuilder timerActionBuilder;
    public static IntakeActionBuilder intakeActionBuilder;

    // States
    StateMachine stateMachine;
    public StateMachine.State intakingPixels;
    public StateMachine.State holdingPixels;
    public StateMachine.State holdingPixelsLow;
    public StateMachine.State idle;
    public StateMachine.State outTakingPixels;
    public StateMachine.State exitingOutTake;


    // Actions
    public Actions empty;

    //TeleOp Actions
    public Actions idleToIntakingPixels, intakingToHoldingPixels, holdingPixelsToIntakingPixels,
            holdingPixelsToIdle, idleToHoldingPixels, holdingPixelsToOutTakingPixels, exitingOutTakeToIdle;

    //Autonomous Actions
    public Actions autoHoldOnePixel, autoOutTakeYellow, autoOutTakeYellowHigh, autoOutTakeYellowLow, autoOpenClaw;


    public static OverrideMotor intakeMotor;
    public static Hanger skyHook;

    public static Button handlerA, handlerB, handlerX, handlerY,
            handlerLeftBumper, handlerRightBumper, handlerLeftTrigger, handlerRightTrigger,
            handlerDPad_Down, handlerDPad_Up, handlerDPad_Left, handlerDPad_Right,
            handlerLeftStick_Up, handlerLeftStick_Down, handlerLeftStick_Left, handlerLeftStick_Right;


    Gamepad gamepad1, gamepad2;
    ElapsedTime teleopTimer;
    public Timer timer;

    boolean isEndgame = false;
    private void updateButtons(){
        handlerA.updateButton(gamepad2);
        handlerB.updateButton(gamepad2);
        handlerX.updateButton(gamepad2);
        handlerY.updateButton(gamepad2);
        handlerLeftBumper.updateButton(gamepad2);
        handlerRightBumper.updateButton(gamepad2);
        handlerLeftTrigger.updateButton(gamepad2);
        handlerRightTrigger.updateButton(gamepad2);
        handlerDPad_Down.updateButton(gamepad2);
        handlerDPad_Up.updateButton(gamepad2);
        handlerDPad_Left.updateButton(gamepad2);
        handlerDPad_Right.updateButton(gamepad2);
        handlerLeftStick_Up.updateButton(gamepad2);
        handlerLeftStick_Down.updateButton(gamepad2);
        handlerLeftStick_Left.updateButton(gamepad2);
        handlerLeftStick_Right.updateButton(gamepad2);
    }

    public void updateSync() {
        stateMachine.updateStateSync(); //Do one call to process potential trigger
    }

    public void update(){
        updateButtons();

        if (teleopTimer.seconds() >= 120 || handlerLeftBumper.Pressed()){
            isEndgame = true;
        }

        if(!(stateMachine.getCurrentState() == outTakingPixels) && isEndgame){
            skyHook.update();
            planeLauncher.update();
        }


        stateMachine.updateState();

        if(stateMachine.getCurrentState() == outTakingPixels){
            lift.update();
        }

        intake.update(!(stateMachine.getCurrentState() == outTakingPixels));

    }

    public StateMachine.State currentState(){
        return stateMachine.getCurrentState();
    }

}