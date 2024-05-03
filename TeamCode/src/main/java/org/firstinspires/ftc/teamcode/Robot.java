package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.tools.Buttons;
import org.firstinspires.ftc.teamcode.tools.LEDRibbons;
import org.firstinspires.ftc.teamcode.tools.OverrideMotor;
import org.firstinspires.ftc.teamcode.tools.PixelsDetection;
import org.firstinspires.ftc.teamcode.tools.stateMachine.Actions;
import org.firstinspires.ftc.teamcode.tools.stateMachine.ClawActionBuilder;
import org.firstinspires.ftc.teamcode.tools.stateMachine.DeadlineAction;
import org.firstinspires.ftc.teamcode.tools.stateMachine.IntakeActionBuilder;
import org.firstinspires.ftc.teamcode.tools.stateMachine.LiftActionBuilder;
import org.firstinspires.ftc.teamcode.tools.stateMachine.StateMachine;

import java.util.concurrent.TimeUnit;
import java.util.function.BooleanSupplier;

public class Robot {
    public Robot(HardwareMap hardwareMap, Gamepad gamepad1, Gamepad gamepad2) {

        // Gamepads
        buttons = new Buttons(gamepad1, gamepad2);

        lift = new Lift(hardwareMap, buttons);
        claw = new Claw(hardwareMap, buttons);
        planeLauncher = new PlaneLauncher(hardwareMap, buttons.handlerX);
        skyHook = new Hanger(hardwareMap, buttons.handlerDPad_Down, buttons.handlerDPad_Up, buttons.handlerY);//hardwareMap.dcMotor.get("skyHookMotor");
        intake = new Intake(hardwareMap, buttons.handlerLeftTrigger, buttons.handlerLeftStick_Up, buttons.handlerLeftStick_Down);

        ledRibbons = new LEDRibbons(hardwareMap);

        DcMotor blinkinPower = hardwareMap.get(DcMotor.class, "blinkinpower");
        blinkinPower.setPower(1);

        // Timer
        teleopDeadline = new Deadline(90, TimeUnit.SECONDS);

        clawActionBuilder = new ClawActionBuilder(claw);
        liftActionBuilder = new LiftActionBuilder(lift);
        intakeActionBuilder = new IntakeActionBuilder(intake);


        createStateMachine();
        createActions();
        createTeleopStateTransitions();

    }


    private void createStateMachine() {
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
        stateMachine.addState(idle);
        stateMachine.addState(outTakingPixels);
        stateMachine.addState(exitingOutTake);

        // Set initial state
        stateMachine.setInitialState(idle);
    }

    private void createActions() {
        empty = new Actions();

        ////////////////
        //// TELEOP ////
        ////////////////
        startIntakingPixels = new Actions()
                .add(clawActionBuilder.setGripPosition(Claw.gripPositions.OPEN))
                .add(intakeActionBuilder.setLEDMode(PixelsDetection.LEDMode.BLINKING))
                .add(DeadlineAction.waitFor(150, TimeUnit.MILLISECONDS))
                .add(liftActionBuilder.startMotorWithEncoder(-1))
                .add(liftActionBuilder.waitUntilLiftTouchDownPressed())
                .add(liftActionBuilder.stopLiftMotor())
                .add(liftActionBuilder.resetLiftMotorEncoder())
                .add(clawActionBuilder.setPitchPosition(Claw.pitchPositions.INTAKE))
                .add(intakeActionBuilder.startIntakeMotorWithNoEncoder(1));

        holdPixels = new Actions()
                .add(intakeActionBuilder.setFlipperServoPosition(Intake.FlipperPosition.UP))
                .add(DeadlineAction.waitFor(150, TimeUnit.MILLISECONDS))
                .add(clawActionBuilder.setGripPosition(Claw.gripPositions.CLOSE))
                .add(DeadlineAction.waitFor(500, TimeUnit.MILLISECONDS))
                .add(intakeActionBuilder.setLEDMode(PixelsDetection.LEDMode.SOLID))
                .add(intakeActionBuilder.startIntakeMotorWithNoEncoder(-1))
                .add(clawActionBuilder.setPitchPosition(Claw.pitchPositions.INTAKE))
                .add(DeadlineAction.waitFor(150, TimeUnit.MILLISECONDS))
                .add(liftActionBuilder.setLiftMotorPositionWithPower(Lift.Position.HOLDING_TELEOP, 1))
                .add(DeadlineAction.waitFor(100, TimeUnit.MILLISECONDS))
                .add(intakeActionBuilder.stopIntakeMotor());

        holdOnePixel = new Actions()
                .add(intakeActionBuilder.setFlipperServoPosition(Intake.FlipperPosition.UP))
                .add(DeadlineAction.waitFor(150, TimeUnit.MILLISECONDS))
                .add(clawActionBuilder.setGripPosition(Claw.gripPositions.CLOSE))
                .add(DeadlineAction.waitFor(500, TimeUnit.MILLISECONDS))
                .add(intakeActionBuilder.setLEDMode(PixelsDetection.LEDMode.SOLID))
                .add(intakeActionBuilder.startIntakeMotorWithNoEncoder(-1))
                .add(clawActionBuilder.setPitchPosition(Claw.pitchPositions.INTAKE))
                .add(DeadlineAction.waitFor(150, TimeUnit.MILLISECONDS))
                .add(liftActionBuilder.setLiftMotorPositionWithPower(Lift.Position.HOLDING_TELEOP, 1))
                .add(DeadlineAction.waitFor(100, TimeUnit.MILLISECONDS))
                .add(intakeActionBuilder.stopIntakeMotor());

        holdingPixelsToIntakingPixels = new Actions()
                .add(clawActionBuilder.setPitchPosition(Claw.pitchPositions.INTAKE))
                .add(clawActionBuilder.setGripPosition(Claw.gripPositions.OPEN))
                .add(liftActionBuilder.startMotorWithEncoder(-1))
                .add(liftActionBuilder.waitUntilLiftTouchDownPressed())
                .add(liftActionBuilder.stopLiftMotor())
                .add(liftActionBuilder.resetLiftMotorEncoder())
                .add(intakeActionBuilder.startIntakeMotorWithNoEncoder(1))
                .add(intakeActionBuilder.setLEDMode(PixelsDetection.LEDMode.BLINKING));

        holdingPixelsToIdle = new Actions()
                .add(clawActionBuilder.setGripPosition(Claw.gripPositions.OPEN));

        idleToHoldingPixels = new Actions()
                .add(clawActionBuilder.setGripPosition(Claw.gripPositions.CLOSE))
                .add(DeadlineAction.waitFor(200, TimeUnit.MILLISECONDS))
                .add(liftActionBuilder.setLiftMotorPositionWithPower(Lift.Position.HOLDING_TELEOP, 1));

        outTake = new Actions()
                .add(liftActionBuilder.setLiftMotorPositionWithPower(Lift.Position.MIN, 1))
                .add(liftActionBuilder.waitForLiftMotorAbovePosition(Lift.Position.MIN))
                .add(clawActionBuilder.setPitchPosition(Claw.pitchPositions.OUTTAKE));

        outTake22 = new Actions()
                .add(liftActionBuilder.setLiftMotorPositionWithPower(Lift.Position.AUTO_MIN_YELLOW_HIGH, 1))
                .add(liftActionBuilder.waitForLiftMotorAbovePosition(Lift.Position.AUTO_MIN_YELLOW_HIGH))
                .add(clawActionBuilder.setPitchPosition(Claw.pitchPositions.OUTTAKE));

        resetOutTake = new Actions()
                .add(clawActionBuilder.setYawPosition(Claw.yawPositions.INTAKE))
                .add(clawActionBuilder.waitForAnalogYawSensorAtPosition(Claw.yawPositions.RESET,5))
                .add(liftActionBuilder.setLiftMotorPositionWithPower(Lift.Position.MIN, 1))
                .add(liftActionBuilder.waitForLiftMotorAbovePosition(Lift.Position.MIN))
                .add(clawActionBuilder.setPitchPosition(Claw.pitchPositions.INTAKE))
                // To get lift going down as fast as possible, bring it down with motor power instead of servo
                // servo will act as maintaining a linear speed and it's slower than just motor power with help of gravity
                .add(clawActionBuilder.waitForAnalogPitchSensorAtPosition(Claw.pitchPositions.RESET, 10))
                .add(liftActionBuilder.startMotorNoEncoder(-1))
                .add(liftActionBuilder.waitUntilLiftTouchDownPressed())
                .add(liftActionBuilder.stopLiftMotor())
                .add(liftActionBuilder.resetLiftMotorEncoder());

        ////////////////
        // AUTONOMOUS //
        ////////////////
        tryIntakeTwoPixels = new Actions()
                .add(intakeActionBuilder.waitForTwoPixelsOrTimeout(5, TimeUnit.SECONDS));

        tryIntakeOnePixel = new Actions()
                .add(startIntakingPixels)
                .add(intakeActionBuilder.waitForOnePixelOrTimeout(5, TimeUnit.SECONDS));

        intakeOn = new Actions()
                .add(startIntakingPixels);

        autoHoldOnePixel = new Actions()
                .add(clawActionBuilder.setGripPosition(Claw.gripPositions.CLOSE_ONE_PIXEL))
                .add(DeadlineAction.waitFor(500, TimeUnit.MILLISECONDS))
                .add(liftActionBuilder.setLiftMotorPositionWithPower(Lift.Position.HOLDING, 1));

        autoOutTakeYellow = new Actions()
                .add(liftActionBuilder.setLiftMotorPositionWithPower(Lift.Position.AUTO_MIN_YELLOW, 1))
                .add(liftActionBuilder.waitForLiftMotorAbovePosition(Lift.Position.AUTO_MIN_YELLOW))
                .add(clawActionBuilder.setPitchPosition(Claw.pitchPositions.OUTTAKE));

        autoOutTakeYellowHigh = new Actions()
                .add(liftActionBuilder.setLiftMotorPositionWithPower(Lift.Position.AUTO_MIN_YELLOW_HIGH, 1))
                .add(liftActionBuilder.waitForLiftMotorAbovePosition(Lift.Position.AUTO_MIN_YELLOW_HIGH))
                .add(clawActionBuilder.setPitchPosition(Claw.pitchPositions.OUTTAKE));

        autoOutTakeYellowLow = new Actions()
                .add(liftActionBuilder.setLiftMotorPositionWithPower(Lift.Position.AUTO_MIN_YELLOW_LOW, 1))
                .add(liftActionBuilder.waitForLiftMotorAbovePosition(Lift.Position.AUTO_MIN_YELLOW_LOW));

        autonomousOpenClawYellow = new Actions()
                .add(clawActionBuilder.setGripPosition(Claw.gripPositions.OPEN_HALFWAY))
                .add(DeadlineAction.waitFor(300, TimeUnit.MILLISECONDS))
                .add(clawActionBuilder.setGripPosition(Claw.gripPositions.OPEN));

        openClaw = new Actions()
                .add(clawActionBuilder.setGripPosition(Claw.gripPositions.OPEN));

        closeClaw = new Actions()
                .add(clawActionBuilder.setGripPosition(Claw.gripPositions.CLOSE));

        outTakeSetClawYawLeftHorizontal = new Actions()
                .add(outTake)
                .add(clawActionBuilder.setYawPosition(Claw.yawPositions.RIGHT_HORIZONTAL));

        outTakeSetClawYawRightHorizontal = new Actions()
                .add(outTake22)
                .add(clawActionBuilder.setYawPosition(Claw.yawPositions.RIGHT_HORIZONTAL));

        outTakeSetClawYawRightSlantedUp = new Actions()
                .add(outTake)
                .add(clawActionBuilder.setYawPosition(Claw.yawPositions.RIGHT_SLANT_UP));

        autoOutTakeSetClawYawVertical = new Actions()
                .add(autoOutTakeYellowHigh)
                .add(clawActionBuilder.setYawPosition(Claw.yawPositions.RESET));
    }


    private void createTeleopStateTransitions() {
        // button triggers
        BooleanSupplier twoPixelsInPossession = intake.pixels::hasTwoPixelsRaw;

        BooleanSupplier alwaysTrue = ()-> true;


        // adding transitions

        // intaking pixels
        idle.addTransitionTo(intakingPixels, buttons.handlerA::Pressed,
                startIntakingPixels);

        intakingPixels.addTransitionTo(idle, buttons.handlerLeftTrigger::Pressed, empty);

//        intakingPixels.addTransitionTo(holdingPixels, twoPixelsInPossession,
//                holdPixels);

        intakingPixels.addTransitionTo(holdingPixels, buttons.handlerA::Pressed,
                holdPixels);

        holdingPixels.addTransitionTo(intakingPixels, buttons.handlerA::Pressed,
                holdingPixelsToIntakingPixels);

        // rejecting pixels
        holdingPixels.addTransitionTo(idle, buttons.handlerLeftTrigger::Pressed,
                holdingPixelsToIdle);

        /*idle.addTransitionTo(holdingPixels, buttons.handlerLeftTrigger::Pressed,
                idleToHoldingPixels);*/

        holdingPixels.addTransitionTo(outTakingPixels, buttons.handlerB::Pressed,
                outTake);

        // in exitingOutTake state, lift cannot be controlled manually
        outTakingPixels.addTransitionTo(exitingOutTake, buttons.handlerB::Pressed,
                empty);

        exitingOutTake.addTransitionTo(idle, alwaysTrue,
                resetOutTake);
    }

    // Functionalities
    public static Lift lift;
    public static Claw claw;
    public static PlaneLauncher planeLauncher;
    public static Intake intake;
    public static LEDRibbons ledRibbons;

    // Action Builders
    public static ClawActionBuilder clawActionBuilder;
    public static LiftActionBuilder liftActionBuilder;
    public static IntakeActionBuilder intakeActionBuilder;

    // States
    StateMachine stateMachine;
    public StateMachine.State intakingPixels;
    public StateMachine.State holdingPixels;
    public StateMachine.State idle;
    public StateMachine.State outTakingPixels;
    public StateMachine.State exitingOutTake;


    // Actions
    public Actions empty;

    //TeleOp Actions
    public Actions startIntakingPixels, holdPixels, holdOnePixel, holdingPixelsToIntakingPixels,
            holdingPixelsToIdle, idleToHoldingPixels, outTake, outTake22, resetOutTake;

    //Autonomous Actions
    public Actions tryIntakeOnePixel, tryIntakeTwoPixels, intakeOn, autoHoldOnePixel, autoOutTakeYellow, autoOutTakeYellowHigh, autoOutTakeYellowLow, autonomousOpenClawYellow,
            openClaw, closeClaw, outTakeSetClawYawLeftHorizontal, outTakeSetClawYawRightHorizontal, outTakeSetClawYawRightSlantedUp, autoOutTakeSetClawYawVertical;


    public static OverrideMotor intakeMotor;
    public static Hanger skyHook;


    Buttons buttons;
    Deadline teleopDeadline;

    boolean isEndgame = false;

    public void updateSync() {
        stateMachine.updateStateSync(); //Do one call to process potential trigger
    }

    public void update(){
        buttons.update();
        if(teleopDeadline.hasExpired() || buttons.handlerLeftBumper.Pressed()) {
            isEndgame = true;
        }

        if(isEndgame){
            skyHook.update();
            planeLauncher.update();
        }

        stateMachine.updateState();
        if(stateMachine.getCurrentState() == outTakingPixels){
            lift.update();
        }
        intake.update(!(stateMachine.getCurrentState() == outTakingPixels));
        ledRibbons.setPattern(intake.pixels.pixel1.pattern, intake.pixels.pixel2.pattern);

    }

    public void wait(long time, TimeUnit timeUnit){
        Deadline deadline = new Deadline(time, timeUnit);
        while(!deadline.hasExpired()){}
    }

    public StateMachine.State currentState(){
        return stateMachine.getCurrentState();
    }

    public void printGraphvizDot() {stateMachine.printGraphvizDot();}

}