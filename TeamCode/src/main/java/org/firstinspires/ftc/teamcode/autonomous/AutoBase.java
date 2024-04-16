package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.ConfigMenu;
import org.firstinspires.ftc.teamcode.ConfigMenuTest;
import org.firstinspires.ftc.teamcode.Intake;
import org.firstinspires.ftc.teamcode.aprilTags.AprilTagPoseDetection;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.roadRunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadRunner.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.roadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.teamProp.TeamPropDetection;
import org.firstinspires.ftc.teamcode.tools.AutoDataStorage;
import org.firstinspires.ftc.teamcode.tools.Buttons;
import org.firstinspires.ftc.teamcode.tools.PIXEL_COLOR;
import org.firstinspires.ftc.teamcode.tools.Robot;
import org.firstinspires.ftc.teamcode.tools.Global;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

//@Autonomous(name="Autonomous Base")
public abstract class AutoBase extends LinearOpMode {

    private Telemetry telemetryA;

    private Follower follower;

    private PathChain empty, purpleDrop, purpleToLeftSideStackSetup, goToBackdropCenterThroughCenterTruss, goToStackSetupThroughCenterTrussFromCenterBackdrop,goToStackSetupThroughCenterTrussFromLeftBackdrop, goToStackSetupThroughCenterTrussFromRightBackdrop, goToBackdropLeftThroughCenterTruss, goToBackdropRightThroughCenterTruss, backdropToLeftSideStack, park;
    private Deadline timer;
    private boolean alreadyCompiled = false;
    Buttons buttons;
    ConfigMenu menu;

    ConfigItems config;

    public enum STACK_LOCATION {
        LEFT,
        CENTER,
        RIGHT,
    }
    public enum TRUSS_LOCATION {
        LEFT,
        CENTER,
        RIGHT,
    }
    public enum PARK_LOCATION {
        LEFT,
        CENTER,
        RIGHT,
    }
    public enum ALLIANCE {
        BLUE,
        RED,
    }
    public enum SIDE {
        NEAR,
        FAR,
    }
    Robot robot;

    protected StandardTrackingWheelLocalizer myLocalizer;
    SampleMecanumDrive drive;

    public static class ConfigItems {
        ALLIANCE alliance = ALLIANCE.BLUE;
        SIDE side = SIDE.NEAR;
        STACK_LOCATION stack_location = STACK_LOCATION.RIGHT;
        TRUSS_LOCATION truss_location = TRUSS_LOCATION.LEFT;
        PARK_LOCATION park_location = PARK_LOCATION.LEFT;
        int numCycles = 0;
        int waitForStack1 = 0;
        int waitForStack2 = 0;
        int waitForStack3 = 0;
        int waitBackdrop1 = 0;
        int waitBackdrop2 = 0;
        int waitBackdrop3 = 0;
    }


    public static class Coordinates {

        //Blue near side
        Pose2d startPose = new Pose2d(12, 62, Math.toRadians(90.0));
        Pose2d leftTeamProp = new Pose2d(20, 38, Math.toRadians(180.0));
        Pose2d centerTeamProp = new Pose2d(12, 32, Math.toRadians(90.00));
        Pose2d rightTeamProp = new Pose2d(9, 32, Math.toRadians(0.00));

        // vectors to set up for backdrop
        Vector2d setupForBackdrop = new Vector2d(30, 50);

        // near side
        Pose2d backdropIntermediateLeft = new Pose2d(47, 43, Math.toRadians(180.00));
        Pose2d backdropIntermediateCenter = new Pose2d(47, 37, Math.toRadians(180.00));
        Pose2d backdropIntermediateRight = new Pose2d(47, 33, Math.toRadians(180.00));

        //Blue backdrop
        Pose2d backdropStrafeForCenter = new Pose2d(51, 45, Math.toRadians(180.00));
        Pose2d backdropLeft = new Pose2d(51, 43, Math.toRadians(180.00));
        Pose2d backdropCenter = new Pose2d(51, 37, Math.toRadians(180.00));
        Pose2d backdropRight = new Pose2d(51, 33, Math.toRadians(180.00));

        // Blue alliance parking
        Pose2d parkIntermediate = new Pose2d(42, 11.5, Math.toRadians(180.00));
        Vector2d parkBetweenBackdrops = new Vector2d(50, 11.5);
        Vector2d parkInCorner = new Vector2d(47, 62);

        //blue far side
        Vector2d prepareFarDropOutside = new Vector2d(-56, 60);
        Vector2d prepareFarDropCenter = new Vector2d(-37, 59);

        Vector2d backdropIntermediateFarOutside = new Vector2d(18, 59);
        Vector2d backdropIntermediateFarStageDoor = new Vector2d(18, 12);


        // Side stacks
        Vector2d purpleDropToStackPreSetup = new Vector2d(-39.0, 45.0);
        Pose2d purpleDropToStackSetup = new Pose2d(-51, 51, Math.toRadians(180));

        Pose2d centerStack = new Pose2d(-56, 24, Math.toRadians(180));
        Pose2d centerStackSetup = new Pose2d(centerStack.getX()+10, centerStack.getY(), centerStack.getHeading());

        Pose2d leftStack = new Pose2d(-57, 36, centerStack.getHeading());
        Pose2d leftStackSetup = new Pose2d(-48, 36, leftStack.getHeading());


        Pose2d rightStack = new Pose2d(centerStack.getX(), centerStack.getY()-12, centerStack.getHeading());
        Pose2d rightStackSetup = new Pose2d(rightStack.getX()+10, rightStack.getY(), rightStack.getHeading());



        Pose2d purpleToStackLeftControlPoint = new Pose2d(-36, 36);

        // To Backdrop
        Pose2d centerTruss = new Pose2d(-14, 36);
        Pose2d centerTrussToBackDropControlPoint = new Pose2d(30, 36);


        public Coordinates(ALLIANCE alliance, SIDE side) {
            AutoDataStorage.redSide = false;

            // Blue alliance far side
            if(alliance == ALLIANCE.BLUE && side == SIDE.FAR){
                startPose = flipToFarSide(startPose);
                Pose2d tempLeftTeamProp = leftTeamProp;
                leftTeamProp = flipToFarSide(rightTeamProp);
                centerTeamProp = flipToFarSide(centerTeamProp);
                rightTeamProp = flipToFarSide(tempLeftTeamProp);

            }

            // Red alliance
            if (alliance == ALLIANCE.RED){
                // Red backdrop
                setupForBackdrop = flipVectorAcrossX(setupForBackdrop);

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

                prepareFarDropCenter = flipVectorAcrossX(prepareFarDropCenter);
                prepareFarDropOutside = flipVectorAcrossX(prepareFarDropOutside);
                backdropIntermediateFarOutside = flipVectorAcrossX(backdropIntermediateFarOutside);
                backdropIntermediateFarStageDoor = flipVectorAcrossX(backdropIntermediateFarStageDoor);

                purpleDropToStackPreSetup = flipVectorAcrossX(purpleDropToStackPreSetup);
                purpleDropToStackSetup = flipAcrossX(purpleDropToStackSetup);

                centerStack = flipAcrossX(centerStack);
                centerStackSetup = flipAcrossX(centerStackSetup);
                leftStack = flipAcrossX(leftStack);
                leftStackSetup = flipAcrossX(leftStackSetup);
                rightStack = flipAcrossX(rightStack);
                rightStackSetup = flipAcrossX(rightStackSetup);

                AutoDataStorage.redSide = true;

                // Near side
                if (side==SIDE.NEAR){
                    startPose = flipAcrossX(startPose);
                    Pose2d tempLeftTeamProp = leftTeamProp;
                    leftTeamProp = flipAcrossX(rightTeamProp);//blue left spike mark is symmetrical to red right spike mark
                    centerTeamProp = flipAcrossX(centerTeamProp);
                    rightTeamProp = flipAcrossX(tempLeftTeamProp);
                }

                // Far side
                if (side==SIDE.FAR){
                    startPose = flipAcrossCenter(startPose);
                    leftTeamProp = flipAcrossCenter(leftTeamProp);
                    centerTeamProp = flipAcrossCenter(centerTeamProp);
                    rightTeamProp = flipAcrossCenter(rightTeamProp);

                    prepareFarDropOutside = flipVectorAcrossX(prepareFarDropOutside);
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
    Coordinates c; //= new Coordinates(true, true); // change values later

    abstract void Setup();

    private enum STACK_POSITIONS{LEFT, CENTER, RIGHT}
    private PathChain intakeFromStack(AutoBase.STACK_POSITIONS position){
        Pose2d setup = new Pose2d(), stack = new Pose2d();
        if (position == AutoBase.STACK_POSITIONS.LEFT){
            stack = c.leftStack;
            setup = c.leftStackSetup;
        }
        if (position == AutoBase.STACK_POSITIONS.CENTER){
            stack = c.centerStack;
            setup = c.centerStackSetup;
        }
        if (position == AutoBase.STACK_POSITIONS.RIGHT){
            stack = c.rightStack;
            setup = c.rightStackSetup;
        }
        return follower.pathBuilder()
                .addPath(new BezierLine(new Point(setup), new Point(stack)))
                .addParametricCallback(0.2, robot.startIntakingPixels.getRunnable())
                .setPathEndVelocityConstraint(5)
                .build();
    }
    @Override
    public void runOpMode() throws InterruptedException {
        Setup();


        Global.telemetry = telemetry;
        robot = new Robot(hardwareMap, gamepad1, gamepad2);

//        TeamPropDetection teamPropDetection = new TeamPropDetection();
        //teamPropDetection.Setup(hardwareMap, telemetry);
        telemetry.setMsTransmissionInterval(50);


        Robot.intake.setIntakeFlipperPosition(Intake.FlipperPosition.UP);


        drive = new SampleMecanumDrive(hardwareMap);
        // hardware map for odometry encoders
        myLocalizer = new StandardTrackingWheelLocalizer(hardwareMap, null, null);

        // start location (coordinate)

        config = new ConfigItems();
        buttons = new Buttons(gamepad1, gamepad2);
        menu = new ConfigMenu(config, buttons);
        // AUDIENCE SIDE TO RIGHT STACK
        // START POSE: new Pose2d(-36.00, 62.00, Math.toRadians(90.00))

        TrajectorySequence audienceSideLeftPurpleToRightStack = drive.trajectorySequenceBuilder(c.startPose)
                .lineTo(new Vector2d(-36.00, 42.00))
                .lineToLinearHeading(new Pose2d(-32.73, 31.07, Math.toRadians(180.00)))
                .lineToLinearHeading(new Pose2d(-46.00, 12.00, Math.toRadians(180.00)))
                .build();

        TrajectorySequence audienceSideMiddlePurpleToRightStack = drive.trajectorySequenceBuilder(c.startPose)
                .lineToLinearHeading(new Pose2d(-40.50, 24.70, Math.toRadians(180.00)))
                .lineTo(new Vector2d(-44.50, 24.70))
                .lineToLinearHeading(new Pose2d(-46.00, 12.00, Math.toRadians(180.00)))
                .build();

        TrajectorySequence audienceSideRightPurpleToRightStack = drive.trajectorySequenceBuilder(new Pose2d(12.00, 62.00, Math.toRadians(90.00)))
                .lineTo(new Vector2d(12.00, 42.00))
                .lineToLinearHeading(new Pose2d(9.31, 32.49, Math.toRadians(0.00)))
                .build();





        // BACKDROP SIDE TO LEFT STACK
        // START POSE: new Pose2d(12.00, 62.00, Math.toRadians(90.00))

        TrajectorySequence backdropSideRightPurple = drive.trajectorySequenceBuilder(c.startPose)
                .lineTo(new Vector2d(12.00, 42.00))
                .lineToLinearHeading(new Pose2d(8.23, 33.81, Math.toRadians(0.00)))
                .lineTo(new Vector2d(15.23, 33.81))
                .lineToLinearHeading(new Pose2d(15.73, 33.81, Math.toRadians(180.0)))
                //.lineToLinearHeading(c.backdropRight)
                .build();

        TrajectorySequence backdropSideCenterPurple = drive.trajectorySequenceBuilder(c.startPose)
                .lineTo(new Vector2d(10.50, 31.50))
                .lineToLinearHeading(new Pose2d(15.50, 38.50, Math.toRadians(90.00)))
                .lineToLinearHeading(new Pose2d(16.00, 38.50, Math.toRadians(180.00)))
                .build();


        TrajectorySequence goToBackdrop = drive.trajectorySequenceBuilder(backdropSideCenterPurple.end())
                .lineToLinearHeading(c.backdropRight)
                .build();



        // Let's have at list 33% chance to pick it right if nothing works
        TeamPropDetection.propLocation propLoc = TeamPropDetection.propLocation.CENTER;



        AprilTagPoseDetection apriltags = new AprilTagPoseDetection();
        apriltags.setup(config.alliance == ALLIANCE.BLUE, hardwareMap);

        apriltags.visionPortal.stopStreaming();
        apriltags.visionPortal.setProcessorEnabled(apriltags.aprilTag, false);
        apriltags.visionPortal.setProcessorEnabled(apriltags.teamPropDetectionPipeline, true);
        apriltags.visionPortal.resumeStreaming();

        // Make sure camera is streaming before we try to set the exposure controls
        if (apriltags.visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            Global.telemetry.addData("Camera", "Waiting");
            Global.telemetry.update();
            while (!isStopRequested() && (apriltags.visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            Global.telemetry.addData("Camera", "Ready");
            Global.telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested())
        {
            ExposureControl exposureControl = apriltags.visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)6, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = apriltags.visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(250);
            sleep(20);
        }


        while (!isStarted() && !isStopRequested())
        {
            // Team prop detection
            TeamPropDetection.propLocation currentPropLoc = apriltags.GetPropLocation();
            if(currentPropLoc!=TeamPropDetection.propLocation.NULL) {
                propLoc = currentPropLoc;
                telemetry.addLine("Detected:" + propLoc);
                telemetry.update();
            }

            // Config menu
            menu.update();

            if(menu.isLocked() && !alreadyCompiled){
                alreadyCompiled = true;

                //Compile all the trajectories using the input menu items
                Coordinates c = new Coordinates(config.alliance, config.side);
                TrajectoryBuilder trajectoryBuilder = new TrajectoryBuilder(c, drive, config);
                ArrayList<TrajectorySequence> finalTrajectory;

            }
            else if (!menu.isLocked()){
                alreadyCompiled = false;
            }


        }

        apriltags.visionPortal.stopStreaming();
        apriltags.visionPortal.setProcessorEnabled(apriltags.aprilTag, true);
        apriltags.visionPortal.setProcessorEnabled(apriltags.teamPropDetectionPipeline, false);
        apriltags.visionPortal.resumeStreaming();


        myLocalizer.setPoseEstimate(c.startPose);
        drive.setPoseEstimate(c.startPose); // !!!!!



//
//        if (propLoc == TeamPropDetection.propLocation.LEFT) {
//            finalTrajectory = trajectoryBuilder.trajectorySequenceLeft;
//        }
//        else if (propLoc == TeamPropDetection.propLocation.CENTER) {
//            finalTrajectory = trajectoryBuilder.trajectorySequenceCenter;
//        }
//        else {



        waitForStart();


        drive.followTrajectorySequence(backdropSideCenterPurple);
        Pose2d correctedPose = apriltags.getRobotPosFromTags();
        drive.setPoseEstimate(correctedPose);
        telemetry.addLine("x: "+correctedPose.getX() + " y: "+correctedPose.getY() + " heading: " + correctedPose.getHeading());
        telemetry.update();

        while(!isStopRequested()){
            // wait
        }
        //drive.followTrajectorySequence(goToBackdrop);

        /*TrajectorySequence untitled0 = drive.trajectorySequenceBuilder(new Pose2d(-36.00, 62.00, Math.toRadians(90.00)))
                .lineToLinearHeading(new Pose2d(-41.01, 21.65, Math.toRadians(0.00)))
                .lineTo(new Vector2d(-39.51, 21.65))
                .splineTo(new Vector2d(-37.58, 7.73), Math.toRadians(270.00))
                .splineTo(new Vector2d(-46.00, 12.00), Math.toRadians(180.00))
                .build();
*/






//        Deadline deadline1 = new Deadline(500, TimeUnit.MILLISECONDS);
//        while(!deadline1.hasExpired()){
//
//        }



        /*robot.intake.setIntakeFlipperPosition(Intake.FlipperPosition.PIXEL5);


        //drive.followTrajectorySequence(stackSetup1);

        robot.startIntakingPixels.runAsync();
        drive.followTrajectorySequence(intakeRightStack);

        robot.tryIntakeTwoPixels.run();


        while(!robot.intake.pixels.hasTwoPixels()) {
            robot.intake.pixels.update();
        }

        robot.intake.setIntakeFlipperPosition(Intake.FlipperPosition.UP);
        robot.holdPixels.run();*/


        //drive.followTrajectorySequence(untitled1);





        /** RR code test, previous code for center stack + 1 cycle

        drive.followTrajectorySequence(purpleDrop);

        robot.intake.setIntakeFlipperPosition(Intake.FlipperPosition.PIXEL5);

        drive.followTrajectorySequence(stackSetup1);

        robot.startIntakingPixels.runAsync();
        drive.followTrajectorySequence(intakeStack1);

        robot.tryIntakeTwoPixels.run();


        while(!robot.intake.pixels.hasTwoPixels()) {
            robot.intake.pixels.update();
        }

        robot.intake.setIntakeFlipperPosition(Intake.FlipperPosition.UP);
        robot.holdPixels.run();

        drive.followTrajectorySequence(goToBackdrop1);
        robot.outTakeSetClawYawRightHorizontal.run();
        Deadline deadline1 = new Deadline(500, TimeUnit.MILLISECONDS);
        while(!deadline1.hasExpired()){

        }

        robot.openClaw.run();
        Deadline deadline2 = new Deadline(250, TimeUnit.MILLISECONDS);
        while(!deadline2.hasExpired()){

        }
        drive.setPoseEstimate(apriltags.getRobotPosFromTags());

        robot.resetOutTake.runAsync();

        drive.followTrajectorySequence(stackSetup2);
        robot.intake.setIntakeFlipperPosition(Intake.FlipperPosition.PIXEL5);
        robot.startIntakingPixels.runAsync();
        drive.followTrajectorySequence(intakeStack2);

        robot.tryIntakeTwoPixels.run();
        robot.intake.setIntakeFlipperPosition(Intake.FlipperPosition.PIXEL4);
        deadline2.reset();
        while(!deadline2.hasExpired()) {
            robot.intake.setIntakeFlipperPosition(Intake.FlipperPosition.PIXEL3);
        }


        while(!robot.intake.pixels.hasTwoPixels()) {
            robot.intake.pixels.update();
        }

        robot.intake.setIntakeFlipperPosition(Intake.FlipperPosition.UP);
        robot.holdPixels.run();

        drive.followTrajectorySequence(goToBackdrop2);
        robot.outTakeSetClawYawRightHorizontal.run();
        deadline1.reset();
        while(!deadline1.hasExpired()){

        }

        robot.openClaw.run();
        deadline2.reset();
        while(!deadline2.hasExpired()){

        }

         **/




        // PREVIOUS CODE

        /*Deadline deadline21 = new Deadline(5, TimeUnit.SECONDS);
        while(!deadline21.hasExpired()){

        }*/




        /*follower.run(purpleDrop);

        follower.run(purpleToLeftSideStackSetup);

        robot.intake.setIntakeFlipperPosition(Intake.FlipperPosition.PIXEL5);

        follower.run(intakeFromStack(AutoBase.STACK_POSITIONS.LEFT));

        robot.tryIntakeTwoPixels.run();

        robot.intake.setIntakeFlipperPosition(Intake.FlipperPosition.UP);


        robot.holdPixels.run();

        follower.run(goToBackdropCenterThroughCenterTruss, true);
        follower.setPose(apriltags.getRobotPosFromTags());

        robot.openClaw.run();



        follower.run(goToStackSetupThroughCenterTrussFromCenterBackdrop, true);*/

    }

}
