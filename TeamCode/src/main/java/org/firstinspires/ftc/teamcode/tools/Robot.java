package org.firstinspires.ftc.teamcode.tools;

import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hanger;
import org.firstinspires.ftc.teamcode.Lift;
import org.firstinspires.ftc.teamcode.tools.StateMachine.ActionBuilder;
import org.firstinspires.ftc.teamcode.tools.StateMachine.Actions;
import org.firstinspires.ftc.teamcode.tools.StateMachine.StateMachine;

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


        lift = new Lift(hardwareMap, gamepad2);

        // Motors
        intakeMotor = new OverrideMotor(hardwareMap.dcMotor.get("intakeMotor"));
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        skyHook = new Hanger(hardwareMap, handlerDPad_Down, handlerDPad_Up, handlerY);//hardwareMap.dcMotor.get("skyHookMotor");


        planeLauncher = hardwareMap.dcMotor.get("planeLauncher");
        planeLauncher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Servos
        planeAngle = hardwareMap.servo.get("planeAngle");
        clawPitch = hardwareMap.servo.get("clawPitch");
        clawYaw = hardwareMap.servo.get("clawYaw");
        clawGrip = hardwareMap.servo.get("clawGrip");

        clawYawAnalogSensor = hardwareMap.get(AnalogInput.class, "rotationPositionInput");
        clawPitchAnalogSensor = hardwareMap.get(AnalogInput.class, "armPositionInput");

        analog_ClawYaw_ResetPosition = 180;
        analog_ClawPitch_ResetPosition = 323;

        planeAngle.scaleRange(0.56, 0.77);
        clawGrip.scaleRange(0, 0.23);
        clawPitch.scaleRange(0.07, 0.28);
        clawYaw.scaleRange(0, 1);

        // Touch Sensors
        liftTouchDown = hardwareMap.touchSensor.get("liftTouchDown");

        clawOpen = 1;
        clawClose = 0.15;
        clawCloseOnePixel = 0;

        clawPitchIntake = 0;
        clawPitchOutTake = 1;

        clawYawIntake = 0.5;
        // Slanted is 60 degrees, allows us to drop pixels vertically for mosaics
        clawYawLeftSlantedUp = 1;
        clawYawLeftHorizontal = clawYawLeftSlantedUp-0.21;
        clawYawLeftSlantedDown = clawYawLeftHorizontal-0.21;

        clawYawRightSlantedUp = 0;
        clawYawRightHorizontal = clawYawRightSlantedUp+0.21;
        clawYawRightSlantedDown = clawYawRightHorizontal+0.21;

        planeAngleStore = 1;
        planeAngleLaunch = 0;

        stateMachine = new StateMachine();

        // Timer
        timer = new ElapsedTime();

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
        empty = new Actions(new ActionBuilder());
        //teleop
        idleToIntakingPixels = new Actions(new ActionBuilder()
                .servoRunToPosition(clawGrip, clawOpen)
                .resetTimer(timer)
                .waitUntil(timer, 150)
                .startMotor(lift.liftMotor, -1, true)
                .waitForTouchSensorPressed(liftTouchDown)
                .stopMotor(lift.liftMotor)
                .resetMotorEncoder(lift.liftMotor)
                .servoRunToPosition(clawPitch, clawPitchIntake)
                .startMotor(intakeMotor, 1, false));

        intakingToHoldingPixels = new Actions(new ActionBuilder()
                .servoRunToPosition(clawGrip, clawClose)
                .stopMotor(intakeMotor)
                .servoRunToPosition(clawPitch, Robot.clawPitchIntake)
                .resetTimer(timer)
                .waitUntil(timer, 150)
                .setMotorPosition(lift.liftMotor, lift.liftEncoderHoldingTeleop, 1));

        holdingPixelsToIntakingPixels = new Actions(new ActionBuilder()
                .servoRunToPosition(clawPitch, clawPitchIntake)
                .servoRunToPosition(clawGrip, clawOpen)
                .startMotor(lift.liftMotor, -1, true)
                .waitForTouchSensorPressed(liftTouchDown)
                .stopMotor(lift.liftMotor)
                .resetMotorEncoder(lift.liftMotor)
                .startMotor(intakeMotor, 1, false));

        holdingPixelsToIdle = new Actions(new ActionBuilder()
                .servoRunToPosition(clawGrip, clawOpen));

        idleToHoldingPixels = new Actions(new ActionBuilder()
                .servoRunToPosition(clawGrip, clawClose)
                .resetTimer(timer)
                .waitUntil(timer, 200)
                .setMotorPosition(lift.liftMotor, lift.liftEncoderHoldingTeleop, 1));

        holdingPixelsToOutTakingPixels = new Actions(new ActionBuilder()
                .setMotorPosition(lift.liftMotor, lift.liftEncoderMin, 1)
                .waitForMotorAbovePosition(lift.liftMotor, lift.liftEncoderMin)
                .servoRunToPosition(clawPitch, clawPitchOutTake));

        exitingOutTakeToIdle = new Actions(new ActionBuilder()
                .servoRunToPosition(clawYaw, clawYawIntake)
                .waitForAnalogSensorAtPosition(clawYawAnalogSensor, analog_ClawYaw_ResetPosition, 5)

                .setMotorPosition(lift.liftMotor, lift.liftEncoderMin, 1)
                .waitForMotorAbovePosition(lift.liftMotor, lift.liftEncoderMin)

                .servoRunToPosition(clawPitch, clawPitchIntake)

                // To get lift going down as fast as possible, bring it down with motor power instead of servo
                // servo will act as maintaining a linear speed and it's slower than just motor power with help of gravity

                .waitForAnalogSensorAtPosition(clawPitchAnalogSensor, analog_ClawPitch_ResetPosition, 10)
                .startMotor(lift.liftMotor, -1, false)
                .waitForTouchSensorPressed(liftTouchDown)
                .stopMotor(lift.liftMotor)
                .resetMotorEncoder(lift.liftMotor));
        //auto

        autoHoldOnePixel = new Actions(new ActionBuilder()
                .servoRunToPosition(clawGrip, clawCloseOnePixel)
                .waitUntil(timer, 500)
                .setMotorPosition(lift.liftMotor, lift.liftEncoderHolding, 1));


        autoOutTakeYellow = new Actions(new ActionBuilder()
                .setMotorPosition(lift.liftMotor, lift.liftEncoderMin+150, 1)
                .waitForMotorAbovePosition(lift.liftMotor, lift.liftEncoderMin+150)
                .servoRunToPosition(clawPitch, clawPitchOutTake));

        autoOutTakeYellowLow = new Actions(new ActionBuilder()
                .setMotorPosition(lift.liftMotor, lift.liftEncoderHoldingLow, 1)
                .waitForMotorAbovePosition(lift.liftMotor, lift.liftEncoderHoldingLow));

        autoOpenClaw = new Actions(new ActionBuilder()
                .servoRunToPosition(clawGrip, clawOpen)
                .resetTimer(timer)
                .waitUntil(timer, 300)
                .servoRunToPosition(clawPitch, clawPitchIntake)
                .resetTimer(timer)
                .waitUntil(timer, 150));


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


    // States
    StateMachine stateMachine;
    public static Lift lift;
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
    public Actions autoHoldOnePixel, autoOutTakeYellow, autoOutTakeYellowLow, autoOpenClaw;

    // Motors

    public static OverrideMotor intakeMotor;
    public static Hanger skyHook;
    public static DcMotor planeLauncher;
    // Servos
    public static Servo clawPitch, clawYaw, clawGrip, planeAngle;
    public static AnalogInput clawYawAnalogSensor, clawPitchAnalogSensor;
    // TouchSensors
    public static TouchSensor liftTouchDown;

    public static double clawPitchIntake, clawPitchOutTake;
    public static double clawOpen, clawClose, clawCloseOnePixel, clawYawIntake,
            clawYawLeftSlantedUp, clawYawLeftHorizontal, clawYawLeftSlantedDown, clawYawRightSlantedUp, clawYawRightHorizontal, clawYawRightSlantedDown;

    public static double planeAngleStore, planeAngleLaunch;

    public static double analog_ClawYaw_ResetPosition, analog_ClawPitch_ResetPosition;

    public static Button handlerA, handlerB, handlerX, handlerY, handlerLeftBumper,
            handlerRightBumper, handlerLeftTrigger, handlerRightTrigger, handlerDPad_Down, handlerDPad_Up, handlerDPad_Left, handlerDPad_Right;

    Gamepad gamepad1, gamepad2;
    ElapsedTime timer;

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
    }

    public void updateSync() {
        stateMachine.updateStateSync(); //Do one call to process potential trigger
    }

    public void update(){
        updateButtons();

        // Manages Reject mode on Roomba as an override of its current power and state
        if(handlerLeftTrigger.Pressed()) {
            intakeMotor.setOverridePower(-1);
        } else if (handlerLeftTrigger.Released()) {
            intakeMotor.cancelOverridePower();
        }
        if(handlerX.On()) {
            planeAngle.setPosition(planeAngleLaunch);
            sleep(500);
            planeLauncher.setPower(1);
            sleep(500);
        }
        else{
            planeAngle.setPosition(planeAngleStore);
            planeLauncher.setPower(0);
        }
        if(!(stateMachine.getCurrentState() == outTakingPixels)){
            skyHook.update(handlerDPad_Down);
        }


        stateMachine.updateState();

        if(stateMachine.getCurrentState() == outTakingPixels){
            lift.update();
        }
    }

    public StateMachine.State currentState(){
        return stateMachine.getCurrentState();
    }

}
