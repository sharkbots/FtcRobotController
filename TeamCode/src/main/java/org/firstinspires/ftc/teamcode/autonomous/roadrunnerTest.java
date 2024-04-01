package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.Claw;
import org.firstinspires.ftc.teamcode.Intake;
import org.firstinspires.ftc.teamcode.roadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadRunner.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.roadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.teamProp.TeamPropDetection;
import org.firstinspires.ftc.teamcode.tools.AutoDataStorage;
import org.firstinspires.ftc.teamcode.tools.Robot;
import org.firstinspires.ftc.teamcode.tools.Global;

import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

//@Autonomous(name="Autonomous Base")
@Autonomous(name = "{rr a}", group = "Autonomous Pathing Tuning")
public class roadrunnerTest extends LinearOpMode {

    protected StandardTrackingWheelLocalizer myLocalizer;
    SampleMecanumDrive drive;
    Robot robot;


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
        Pose2d stackCenter = new Pose2d(-56, 24, Math.toRadians(180));
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
    PedroTest.Coordinates c = new PedroTest.Coordinates(true, false);

    public void setup(){
        Global.telemetry = telemetry;
        robot = new Robot(hardwareMap, gamepad1, gamepad2);

        telemetry.setMsTransmissionInterval(50);

        drive = new SampleMecanumDrive(hardwareMap);
        // hardware map for odometry encoders
        myLocalizer = new StandardTrackingWheelLocalizer(hardwareMap, null, null);
        // start location (coordinate)
    }

    @Override
    public void runOpMode() throws InterruptedException {
        setup();

        TrajectorySequence purpleDrop = drive.trajectorySequenceBuilder(c.startPose)
                .lineToLinearHeading(c.centerTeamProp)
                .build();

        TrajectorySequence stackSetup1 = drive.trajectorySequenceBuilder(purpleDrop.end())
                .splineToLinearHeading(c.stackLeftSetup, Math.toRadians(180))
                .build();

        TrajectorySequence intakeStack1 = drive.trajectorySequenceBuilder(stackSetup1.end())
                .lineToLinearHeading(c.stackLeft)
                .build();

        TrajectorySequence goToBackdrop1 = drive.trajectorySequenceBuilder(intakeStack1.end())
                .lineToLinearHeading(c.backdropCenter)
                .build();

        myLocalizer.setPoseEstimate(c.startPose);
        drive.setPoseEstimate(c.startPose); // !!!!!

        waitForStart();

        drive.followTrajectorySequence(purpleDrop);

        robot.intake.setIntakeFlipperPosition(Intake.FlipperPosition.PIXEL5);

        drive.followTrajectorySequence(stackSetup1);

        robot.tryIntakeTwoPixels.runAsync();

        drive.followTrajectorySequence(intakeStack1);

        while(!robot.intake.pixels.hasTwoPixels()) {
        }

        robot.intake.setIntakeFlipperPosition(Intake.FlipperPosition.UP);
        robot.holdPixels.run();

        drive.followTrajectorySequence(goToBackdrop1);
        robot.outTake.run();
        Deadline deadline = new Deadline(500, TimeUnit.MILLISECONDS);
        while(!deadline.hasExpired()){

        }

        robot.openClaw.run();


        while(!isStopRequested()){

        }





    }


}
