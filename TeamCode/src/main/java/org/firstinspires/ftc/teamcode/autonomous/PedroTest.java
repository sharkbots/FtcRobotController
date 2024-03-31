package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.Intake;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.tools.AutoDataStorage;
import org.firstinspires.ftc.teamcode.tools.Global;
import org.firstinspires.ftc.teamcode.tools.Robot;

import java.util.concurrent.TimeUnit;


/**
 * This is the Circle autonomous OpMode. It runs the robot in a PathChain that's actually not quite
 * a circle, but some Bezier curves that have control points set essentially in a square. However,
 * it turns enough to tune your centripetal force correction and some of your heading. Some lag in
 * heading is to be expected.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/12/2024
 */
@Config
@Autonomous (name = "{Pedro Testing}", group = "Autonomous Pathing Tuning")
public class PedroTest extends LinearOpMode {
    private Telemetry telemetryA;

    public static double RADIUS = 10;

    private Follower follower;

    private PathChain empty, purpleDrop, purpleToLeftSideStackSetup, goToBackdropCenterThroughCenterTruss, goToStackSetupThroughCenterTrussFromCenterBackdrop,goToStackSetupThroughCenterTrussFromLeftBackdrop, goToStackSetupThroughCenterTrussFromRightBackdrop, goToBackdropLeftThroughCenterTruss, goToBackdropRightThroughCenterTruss, backdropToLeftSideStack, park;
    private Deadline timer;

    public static class Coordinates{
        Boolean isBlueAlliance;
        Boolean isNearSide;

        //Blue near side
        Pose2d startPose = new Pose2d(12, 62, Math.toRadians(90.0));
        Pose2d leftTeamProp = new Pose2d(20, 38, Math.toRadians(90.0));
        Pose2d centerTeamProp = new Pose2d(12, 32, Math.toRadians(90.00));
        Pose2d rightTeamProp = new Pose2d(9, 32, Math.toRadians(0.00));

        // near side
        Pose2d backdropIntermediateLeft = new Pose2d(47, 43, Math.toRadians(180.00));
        Pose2d backdropIntermediateCenter = new Pose2d(47, 37, Math.toRadians(180.00));
        Pose2d backdropIntermediateRight = new Pose2d(47, 33, Math.toRadians(180.00));

        //Blue backdrop
        Pose2d backdropStrafeForCenter = new Pose2d(51, 45, Math.toRadians(180.00));
        Pose2d backdropLeft = new Pose2d(51, 41, Math.toRadians(180.00));
        Pose2d backdropCenter = new Pose2d(51, 34, Math.toRadians(180.00));
        Pose2d backdropRight = new Pose2d(51, 31, Math.toRadians(180.00));

        // Blue alliance parking
        Pose2d parkIntermediate = new Pose2d(42, 11.5, Math.toRadians(180.00));
        Vector2d parkBetweenBackdrops = new Vector2d(50, 11.5);
        Vector2d parkInCorner = new Vector2d(47, 62);

        // Side stacks
        Pose2d stackCenter = new Pose2d(-58, 24, Math.toRadians(180));
        Pose2d stackCenterSetup = new Pose2d(stackCenter.getX()+10, stackCenter.getY(), stackCenter.getHeading());

        Pose2d stackLeft = new Pose2d(stackCenter.getX(), stackCenter.getY()+12, stackCenter.getHeading());
        Pose2d stackLeftSetup = new Pose2d(stackLeft.getX()+10, stackLeft.getY(), stackLeft.getHeading());

        Pose2d stackRight = new Pose2d(stackCenter.getX(), stackCenter.getY()-12, stackCenter.getHeading());
        Pose2d stackRightSetup = new Pose2d(stackRight.getX()+10, stackRight.getY(), stackRight.getHeading());



        Pose2d purpleToStackLeftControlPoint = new Pose2d(-36, 36);

        // To Backdrop
        Pose2d centerTruss = new Pose2d(-14, 36);
        Pose2d centerTrussToBackDropControlPoint = new Pose2d(30, 36);


        public Coordinates(Boolean isBlueAlliance, Boolean isNearSide) {
            this.isBlueAlliance = isBlueAlliance;
            this.isNearSide = isNearSide;
            AutoDataStorage.redSide = false;

            // Default is blue alliance near side

            // Blue alliance far side
            if(isBlueAlliance && !isNearSide){
                startPose = flipToFarSide(startPose);
                Pose2d tempLeftTeamProp = leftTeamProp;
                leftTeamProp = flipToFarSide(rightTeamProp);
                centerTeamProp = flipToFarSide(centerTeamProp);
                rightTeamProp = flipToFarSide(tempLeftTeamProp);

            }

            // Red alliance
            if (!isBlueAlliance){
                // Red backdrop
                Pose2d tempLeftPose = backdropIntermediateLeft;
                backdropIntermediateLeft = flipAcrossX(backdropIntermediateRight);
                backdropIntermediateCenter = flipAcrossX(backdropIntermediateCenter);
                backdropIntermediateRight = flipAcrossX(tempLeftPose);

                tempLeftPose = backdropLeft;
                backdropLeft = flipAcrossX(backdropRight);
                backdropCenter = flipAcrossX(backdropCenter);
                backdropStrafeForCenter = flipAcrossX(backdropStrafeForCenter);
                backdropRight = flipAcrossX(tempLeftPose);

                parkIntermediate = flipAcrossX(parkIntermediate);
                parkInCorner = flipVectorAcrossX(parkInCorner);
                parkBetweenBackdrops = flipVectorAcrossX(parkBetweenBackdrops);

                centerTrussToBackDropControlPoint = flipAcrossX(centerTrussToBackDropControlPoint);

                stackLeft = flipAcrossX(stackLeft);


                AutoDataStorage.redSide = true;

                // Near side
                if (isNearSide){
                    startPose = flipAcrossX(startPose);
                    Pose2d tempLeftTeamProp = leftTeamProp;
                    leftTeamProp = flipAcrossX(rightTeamProp);//blue left spike mark is symmetrical to red right spike mark
                    centerTeamProp = flipAcrossX(centerTeamProp);
                    rightTeamProp = flipAcrossX(tempLeftTeamProp);
                }

                // Far side
                if (!isNearSide){
                    startPose = flipAcrossCenter(startPose);
                    leftTeamProp = flipAcrossCenter(leftTeamProp);
                    centerTeamProp = flipAcrossCenter(centerTeamProp);
                    rightTeamProp = flipAcrossCenter(rightTeamProp);
                }
            }
        }


        // Blue alliance to red alliance
        public Pose2d flipAcrossX(Pose2d pose){
            return new Pose2d(pose.getX(), -pose.getY(), (-pose.getHeading())%Math.toRadians(360));
        }

        public Vector2d flipVectorAcrossX(Vector2d vector2d){
            return new Vector2d(vector2d.getX(), -vector2d.getY());
        }

        // Near side to far side
        public Pose2d flipToFarSide(Pose2d pose){
            return new Pose2d(-(pose.getX()+24), pose.getY(), (Math.toRadians(180)-pose.getHeading())%Math.toRadians(360));
        }

        public Pose2d flipAcrossCenter(Pose2d pose) {
            return flipAcrossX(flipToFarSide(pose));
        }

    }
    PedroTest.Coordinates c = new Coordinates(true, false);
    Robot robot;

    public void setup() {
        Global.telemetry = telemetry;
        robot = new Robot(hardwareMap, gamepad1, gamepad2);
        Robot.intake.setIntakeFlipperPosition(Intake.FlipperPosition.UP);

        follower = new Follower(hardwareMap);

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        follower.setStartingPose(c.startPose);

        purpleDrop = follower.pathBuilder()
                .addPath(new BezierLine(new Point(c.startPose), new Point(c.centerTeamProp)))
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .build();

        purpleToLeftSideStackSetup = follower.pathBuilder()
                .addPath(new BezierCurve(new Point(c.centerTeamProp), new Point(c.purpleToStackLeftControlPoint), new Point(c.stackLeftSetup)))
                .setPathEndHeadingConstraint(Math.toRadians(180))
                .build();

        goToBackdropCenterThroughCenterTruss = goToBackdropThroughCenterTruss(c.backdropCenter);
        goToBackdropLeftThroughCenterTruss = goToBackdropThroughCenterTruss(c.backdropLeft);
        goToBackdropRightThroughCenterTruss = goToBackdropThroughCenterTruss(c.backdropRight);

        goToStackSetupThroughCenterTrussFromCenterBackdrop = goToStackSetupThroughCenterTruss(c.backdropCenter);
        goToStackSetupThroughCenterTrussFromLeftBackdrop = goToStackSetupThroughCenterTruss(c.backdropLeft);
        goToStackSetupThroughCenterTrussFromRightBackdrop = goToStackSetupThroughCenterTruss(c.backdropRight);

        telemetryA.addLine("Good to start, go for it.");
        telemetryA.update();
        Global.telemetry.speak("THE ROBOT IS ALIVE");

    }

    private PathChain goToBackdropThroughCenterTruss(Pose2d backdropPosition) {
        return follower.pathBuilder()
                .addPath(new BezierLine(new Point(c.stackLeft), new Point(c.centerTruss)))
                .addParametricCallback(0.8, robot.outTake.getAsyncRunnable())
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(new BezierCurve(new Point(c.centerTruss), new Point(c.centerTrussToBackDropControlPoint), new Point(backdropPosition)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
    }

    private PathChain goToStackSetupThroughCenterTruss(Pose2d backdropPosition) {
        return follower.pathBuilder()
                .addPath(new BezierCurve(new Point(backdropPosition), new Point(c.centerTrussToBackDropControlPoint), new Point(c.centerTruss)))
                .addParametricCallback(0.1, robot.resetOutTake.getAsyncRunnable())
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(new BezierLine(new Point(c.centerTruss), new Point(c.stackLeftSetup)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
    }

    private enum STACK_POSITIONS{LEFT, CENTER, RIGHT}
    private PathChain intakeFromStack(STACK_POSITIONS position){
        Pose2d setup = new Pose2d(), stack = new Pose2d();
        if (position == STACK_POSITIONS.LEFT){
            stack = c.stackLeft;
            setup = c.stackLeftSetup;
        }
        if (position == STACK_POSITIONS.CENTER){
            stack = c.stackCenter;
            setup = c.stackCenterSetup;
        }
        if (position == STACK_POSITIONS.RIGHT){
            stack = c.stackRight;
            setup = c.stackRightSetup;
        }
        return follower.pathBuilder()
                .addPath(new BezierLine(new Point(setup), new Point(stack)))
                .addParametricCallback(0.2, robot.tryIntakeTwoPixels.getAsyncRunnable())
                .addPath(new BezierLine(new Point(stack), new Point(c.centerTrussToBackDropControlPoint)))
                .setPathEndVelocityConstraint(5)
                .build();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        setup();

        waitForStart();
        //Deadline dead = new Deadline(1, TimeUnit.SECONDS);
        follower.run(purpleDrop);

        follower.run(purpleToLeftSideStackSetup);

        robot.intake.setIntakeFlipperPosition(Intake.FlipperPosition.PIXEL5);

        follower.run(intakeFromStack(STACK_POSITIONS.LEFT), true);

        robot.intake.setIntakeFlipperPosition(Intake.FlipperPosition.UP);

        robot.holdPixels.run();
//
//
//        follower.update();
//        follower.telemetryDebug(telemetryA);
//        follower.update();

        follower.run(goToBackdropCenterThroughCenterTruss, true);

        robot.openClaw.run();


        follower.run(goToStackSetupThroughCenterTrussFromLeftBackdrop, true);



        while(!isStopRequested()){

        }

        //robot.openClaw.run();
        //robot.resetOutTake.run();
        //follower.run(goToStackSetupThroughCenterTrussFromCenterBackdrop);

        //Robot.intake.setIntakeFlipperPosition(Intake.FlipperPosition.PIXEL5);
        //follower.run(intakeFromStack(STACK_POSITIONS.LEFT));
        //Robot.intake.setIntakeFlipperPosition(Intake.FlipperPosition.UP);
        //follower.run(goToBackdropRightThroughCenterTruss);
        //follower.run(goToStackSetupThroughCenterTrussFromRightBackdrop);

        //follower.run(intakeFromStack(STACK_POSITIONS.LEFT));
        //follower.run(goToBackdropLeftThroughCenterTruss);
        //follower.run(goToStackSetupThroughCenterTrussFromLeftBackdrop);

/*
        // first cycle (yellow preload + white from stack)
        Robot.intake.setIntakeFlipperPosition(Intake.FlipperPosition.PIXEL5);
        do {
            robot.tryIntakeTwoPixels.run();
        } while(!Robot.intake.pixels.hasTwoPixels());

        robot.holdPixels.run();
        // go to backdrop

        robot.outTake.run();
        Robot.claw.setGripPosition(Claw.gripPositions.OPEN);
        Deadline waitFarSide = new Deadline(1, TimeUnit.SECONDS);
        //noinspection StatementWithEmptyBody
        while(!waitFarSide.hasExpired()) {
        }
        robot.resetOutTake.run();


        // second cycle (2 whites from stack)
        Robot.intake.setIntakeFlipperPosition(Intake.FlipperPosition.PIXEL4);
        robot.startIntakingPixels.run();
        Robot.intake.setIntakeFlipperPosition(Intake.FlipperPosition.PIXEL3);
        do {
            robot.tryIntakeTwoPixels.run();
        } while(!Robot.intake.pixels.hasTwoPixels());
        robot.holdPixels.run();
        robot.outTake.run();
        Robot.claw.setGripPosition(Claw.gripPositions.OPEN);
        waitFarSide.reset();
        //noinspection StatementWithEmptyBody
        while(!waitFarSide.hasExpired()) {
        }
        robot.resetOutTake.run();



        // third cycle (2 whites from stack)
        Robot.intake.setIntakeFlipperPosition(Intake.FlipperPosition.UP);
        robot.startIntakingPixels.run();
        do {
            robot.tryIntakeTwoPixels.run();
        } while(!Robot.intake.pixels.hasTwoPixels());
        robot.holdPixels.run();
        robot.outTake.run();
        Robot.claw.setGripPosition(Claw.gripPositions.OPEN);
        waitFarSide.reset();
        //noinspection StatementWithEmptyBody
        while(!waitFarSide.hasExpired()) {
        }
        robot.resetOutTake.run();


*/
    }





}
