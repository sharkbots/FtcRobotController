package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.teamcode.Claw;
import org.firstinspires.ftc.teamcode.ConfigMenu;
import org.firstinspires.ftc.teamcode.Intake;
import org.firstinspires.ftc.teamcode.aprilTags.AprilTagPoseDetection;
import org.firstinspires.ftc.teamcode.roadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadRunner.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.roadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.teamProp.TeamPropDetection;
import org.firstinspires.ftc.teamcode.tools.AutoDataStorage;
import org.firstinspires.ftc.teamcode.tools.Buttons;
import org.firstinspires.ftc.teamcode.tools.Robot;
import org.firstinspires.ftc.teamcode.tools.Global;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

@Autonomous(name="Autonomous")
public class AutoBase extends LinearOpMode {

    private Deadline timer;
    private boolean alreadyCompiled = false;
    Buttons buttons;
    ConfigMenu menu;
    ArrayList<TrajectorySequence> finalTrajectory;
    TrajectoryBuilder trajectoryBuilder;

    ConfigItems config;

    public enum STACK_LOCATION {
        LEFT,
        CENTER,
        RIGHT,
    }
    public enum TRUSS_LOCATION {
        DOOR,
        CENTER,
        OUTER,
    }
    public enum PARK_LOCATION {
        CORNER,
        CENTER,
        BETWEEN_BACKDROPS,
    }
    public enum ALLIANCE {
        BLUE,
        RED,
    }
    public enum SIDE {
        BACKDROP,
        AUDIENCE,
    }
    Robot robot;

    protected StandardTrackingWheelLocalizer myLocalizer;
    SampleMecanumDrive drive;

    public static class ConfigItems {
        ALLIANCE alliance = ALLIANCE.BLUE;
        SIDE side = SIDE.BACKDROP;
        STACK_LOCATION stack_location = STACK_LOCATION.RIGHT;
        TRUSS_LOCATION truss_location = TRUSS_LOCATION.DOOR;
        PARK_LOCATION park_location = PARK_LOCATION.CORNER;
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
        double driftOffsetWhiteStack = -3.0; // Inaara: Built below in Constructor, not here anymore

        Pose2d prepareToGoToStageDoor;// = new Pose2d(-38.78, 11+driftOffsetWhiteStack, Math.toRadians(180));
        Pose2d prepareToGoToStageDoor2;// = new Pose2d(-12, 11+driftOffsetWhiteStack, Math.toRadians(180.0));
        Pose2d intermediateCyclePose;// = new Pose2d(37.0, 11+driftOffsetWhiteStack, Math.toRadians(180.0));


        Pose2d prepareToGoToBackdropCycle = new Pose2d(37.0, 35.0, Math.toRadians(180.0));
        Vector2d spatialMarkerGoToBackdrop = new Vector2d(12, 11);

        // near side
        Pose2d backdropIntermediateLeft = new Pose2d(47, 43, Math.toRadians(180.00));
        Pose2d backdropIntermediateCenter = new Pose2d(47, 37, Math.toRadians(180.00));
        Pose2d backdropIntermediateRight = new Pose2d(47, 33, Math.toRadians(180.00));

        //Blue backdrop
        Pose2d backdropStrafeForCenter = new Pose2d(51, 45, Math.toRadians(180.00));
        Pose2d backdropLeft = new Pose2d(50.5, 41.41, Math.toRadians(180.00));
        Pose2d backdropCenter = new Pose2d(50.5, 35.41, Math.toRadians(180.00));
        Pose2d backdropRight = new Pose2d(50.5, 29.41, Math.toRadians(180.00));

        // Blue alliance parking
        Pose2d parkIntermediate = new Pose2d(42, 11.5, Math.toRadians(180.00));

        Vector2d parkBetweenBackdropsSetup = new Vector2d(45, 11.5);
        Vector2d parkBetweenBackdrops = new Vector2d(60, 7);

        Vector2d parkInCornerSetup = new Vector2d(45, 62);
        Vector2d parkInCorner = new Vector2d(60, 62);


        //blue far side
        Vector2d prepareFarDropOutside = new Vector2d(-56, 60);
        Vector2d prepareFarDropCenter = new Vector2d(-37, 59);

        Vector2d backdropIntermediateFarOutside = new Vector2d(18, 59);
        Vector2d backdropIntermediateFarStageDoor = new Vector2d(18, 12);


        // Side stacks
        Vector2d purpleDropToStackPreSetup = new Vector2d(-39.0, 45.0);
        Pose2d purpleDropToStackSetup = new Pose2d(-51, 51, Math.toRadians(180));

        Pose2d centerStack = new Pose2d(-58.5, 24, Math.toRadians(180));
        Pose2d centerStackSetup = new Pose2d(centerStack.getX()+10, centerStack.getY(), centerStack.getHeading());

        Pose2d leftStack = new Pose2d(-57, 36, centerStack.getHeading());
        Pose2d leftStackSetup = new Pose2d(-48, 36, leftStack.getHeading());


        Pose2d rightStack = new Pose2d(centerStack.getX(), centerStack.getY()-12, centerStack.getHeading());
        Pose2d rightStackSetup = new Pose2d(rightStack.getX()+10, rightStack.getY(), rightStack.getHeading());


        // AUDIENCE SIDE PURPLE
        Vector2d audienceSideLeftPurpleToRightStackCoordinateA = new Vector2d(-36.00, 42.00);
        Pose2d audienceSideLeftPurpleToRightStackCoordinateB = new Pose2d(-32.73, 31.07, Math.toRadians(180.00));

        Pose2d audienceSideMiddlePurpleToRightStackCoordinateA = new Pose2d(-40.50, 24.70, Math.toRadians(180.00));
        Vector2d audienceSideMiddlePurpleToRightStackCoordinateB = new Vector2d(-44.50, 24.70);

        Pose2d audienceSideRightPurpleToRightStackCoordinateA = new Pose2d(-41.01, 21.65, Math.toRadians(0.00));
        Pose2d audienceSideRightPurpleToRightStackCoordinateB = new Pose2d(-34.77, 14.05, 270.0);

        // BACKDROP SIDE PURPLE
        Pose2d backdropSideLeftPurpleCoordinateA = new Pose2d(16.00, 29.00, Math.toRadians(180.00));
        Pose2d backdropSideLeftPurpleCoordinateB = new Pose2d(13.00, 29.00, Math.toRadians(180.00));
        Pose2d backdropSideLeftPurpleCoordinateC = new Pose2d(13.00, 45.00, Math.toRadians(180.00));

        Vector2d backdropSideCenterPurpleCoordinateA = new Vector2d(10.50, 31.50);
        Pose2d backdropSideCenterPurpleCoordinateB = new Pose2d(15.50, 38.50, Math.toRadians(90.00));
        Pose2d backdropSideCenterPurpleCoordinateC = new Pose2d(16.00, 38.50, Math.toRadians(180.00));

        Vector2d backdropSideRightPurpleCoordinateA = new Vector2d(12.00, 42.00);
        Pose2d backdropSideRightPurpleCoordinateB = new Pose2d(10.23, 33.81, Math.toRadians(0.00));
        Vector2d backdropSideRightPurpleCoordinateC = new Vector2d(15.23, 33.81);
        Pose2d backdropSideRightPurpleCoordinateD = new Pose2d(15.73, 33.81, Math.toRadians(180.0));

        Pose2d purpleToStackLeftControlPoint = new Pose2d(-36, 36);

        // To Backdrop
        Pose2d centerTruss = new Pose2d(-14, 36);
        Pose2d centerTrussToBackDropControlPoint = new Pose2d(30, 36);


        public Coordinates(ALLIANCE alliance, SIDE side) {
            AutoDataStorage.redSide = false;

            driftOffsetWhiteStack = (alliance == ALLIANCE.BLUE)?-3.0 : 2.0;
            prepareToGoToStageDoor = new Pose2d(-38.78, 11+driftOffsetWhiteStack, Math.toRadians(180));
            prepareToGoToStageDoor2 = new Pose2d(-12, 11+driftOffsetWhiteStack, Math.toRadians(180.0));
            intermediateCyclePose = new Pose2d(37.0, 11+driftOffsetWhiteStack, Math.toRadians(180.0));


            // Blue alliance far side
            if(alliance == ALLIANCE.BLUE && side == SIDE.AUDIENCE){
                startPose = flipToFarSide(startPose);
                Pose2d tempLeftTeamProp = leftTeamProp;
                leftTeamProp = flipToFarSide(rightTeamProp);
                centerTeamProp = flipToFarSide(centerTeamProp);
                rightTeamProp = flipToFarSide(tempLeftTeamProp);

            }

            // Red alliance
            if (alliance == ALLIANCE.RED){
                // Red backdrop

                Pose2d tempLeftPose = backdropIntermediateLeft;
                backdropIntermediateLeft = flipAcrossX(backdropIntermediateRight);
                backdropIntermediateCenter = flipAcrossX(backdropIntermediateCenter);
                backdropIntermediateRight = flipAcrossX(tempLeftPose);

                backdropLeft = flipAcrossX(backdropLeft);
                backdropCenter = flipAcrossX(backdropCenter);
                backdropStrafeForCenter = flipAcrossX(backdropStrafeForCenter);
                backdropRight = flipAcrossX(backdropRight);

                parkIntermediate = flipAcrossX(parkIntermediate);
                parkInCorner = flipVectorAcrossX(parkInCorner);
                parkInCornerSetup = flipVectorAcrossX(parkInCornerSetup);
                parkBetweenBackdropsSetup = flipVectorAcrossX(parkBetweenBackdropsSetup);
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

                prepareToGoToStageDoor = flipAcrossX(prepareToGoToStageDoor);
                prepareToGoToStageDoor2 = flipAcrossX(prepareToGoToStageDoor2);
                intermediateCyclePose = flipAcrossX(intermediateCyclePose);
                prepareToGoToBackdropCycle = flipAcrossX(prepareToGoToBackdropCycle);
                spatialMarkerGoToBackdrop = flipVectorAcrossX(spatialMarkerGoToBackdrop);


                AutoDataStorage.redSide = true;

                // Near side (Backdrop side)
                if (side==SIDE.BACKDROP){
                    startPose = flipAcrossX(startPose);
                    Pose2d tempLeftTeamProp = leftTeamProp;
                    leftTeamProp = flipAcrossX(rightTeamProp);//blue left spike mark is symmetrical to red right spike mark
                    centerTeamProp = flipAcrossX(centerTeamProp);
                    rightTeamProp = flipAcrossX(tempLeftTeamProp);

                    backdropSideLeftPurpleCoordinateA = flipAcrossX(backdropSideLeftPurpleCoordinateA);
                    backdropSideLeftPurpleCoordinateB = flipAcrossX(backdropSideLeftPurpleCoordinateB);
                    backdropSideLeftPurpleCoordinateC = flipAcrossX(backdropSideLeftPurpleCoordinateC);

                    backdropSideCenterPurpleCoordinateA = flipVectorAcrossX(backdropSideCenterPurpleCoordinateA);
                    backdropSideCenterPurpleCoordinateB = flipAcrossX(backdropSideCenterPurpleCoordinateB);
                    backdropSideCenterPurpleCoordinateC = flipAcrossX(backdropSideCenterPurpleCoordinateC);

                    backdropSideRightPurpleCoordinateA = flipVectorAcrossX(backdropSideRightPurpleCoordinateA);
                    backdropSideRightPurpleCoordinateB = flipAcrossX(backdropSideRightPurpleCoordinateB);
                    backdropSideRightPurpleCoordinateC = flipVectorAcrossX(backdropSideRightPurpleCoordinateC);
                    backdropSideRightPurpleCoordinateD = flipAcrossX(backdropSideRightPurpleCoordinateD);
                }

                // Far side (Audience side)
                if (side==SIDE.AUDIENCE){
                    startPose = flipAcrossCenter(startPose);
                    leftTeamProp = flipAcrossCenter(leftTeamProp);
                    centerTeamProp = flipAcrossCenter(centerTeamProp);
                    rightTeamProp = flipAcrossCenter(rightTeamProp);

                    prepareFarDropOutside = flipVectorAcrossX(prepareFarDropOutside);

                    audienceSideLeftPurpleToRightStackCoordinateA = flipVectorAcrossX(audienceSideLeftPurpleToRightStackCoordinateA);
                    audienceSideLeftPurpleToRightStackCoordinateB = flipAcrossX(audienceSideLeftPurpleToRightStackCoordinateB);
                    audienceSideMiddlePurpleToRightStackCoordinateA = flipAcrossX(audienceSideMiddlePurpleToRightStackCoordinateA);
                    audienceSideMiddlePurpleToRightStackCoordinateB = flipVectorAcrossX(audienceSideMiddlePurpleToRightStackCoordinateB);
                    audienceSideRightPurpleToRightStackCoordinateA = flipAcrossX(audienceSideRightPurpleToRightStackCoordinateA);
                    audienceSideRightPurpleToRightStackCoordinateB = flipAcrossX(audienceSideRightPurpleToRightStackCoordinateB);
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

    public void Setup(){};

    @Override
    public void runOpMode() throws InterruptedException {
        Setup();


        Global.telemetry = telemetry;
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
        robot = new Robot(hardwareMap, gamepad1, gamepad2);

//        TeamPropDetection teamPropDetection = new TeamPropDetection();
        //teamPropDetection.Setup(hardwareMap, telemetry);
        telemetry.setMsTransmissionInterval(50);


        Robot.intake.setIntakeFlipperPosition(Intake.FlipperPosition.UP);

        Robot.claw.setPitchPosition(Claw.pitchPositions.INTAKE);
        Robot.claw.setGripPosition(Claw.gripPositions.CLOSE_ONE_PIXEL);
        Robot.claw.setYawPosition(Claw.yawPositions.INTAKE);




        drive = new SampleMecanumDrive(hardwareMap);
        // hardware map for odometry encoders

        // start location (coordinate)

        config = new ConfigItems();
        buttons = new Buttons(gamepad1, gamepad2);
        menu = new ConfigMenu(config, buttons);
        // AUDIENCE SIDE TO RIGHT STACK
        // START POSE: new Pose2d(-36.00, 62.00, Math.toRadians(90.00))



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
            // Config menu
            menu.update();

            if(menu.isLocked() && !alreadyCompiled){
                alreadyCompiled = true;

                //Compile all the trajectories using the input menu items
                c = new Coordinates(config.alliance, config.side);
                trajectoryBuilder = new TrajectoryBuilder(c, drive, config, robot);
            }
            else if (!menu.isLocked()){
                alreadyCompiled = false;
            }
            if(menu.isLocked()){
                // Team prop detection
                TeamPropDetection.propLocation currentPropLoc = apriltags.GetPropLocation();
                if(currentPropLoc!=TeamPropDetection.propLocation.NULL) {
                    propLoc = currentPropLoc;
                    telemetry.addData("Prop", propLoc);
                }
            }
        }

        apriltags.visionPortal.stopStreaming();
        apriltags.visionPortal.setProcessorEnabled(apriltags.aprilTag, true);
        apriltags.visionPortal.setProcessorEnabled(apriltags.teamPropDetectionPipeline, false);
        apriltags.visionPortal.resumeStreaming();


        if(config.alliance == ALLIANCE.RED){
            if (propLoc == TeamPropDetection.propLocation.LEFT) {
                finalTrajectory = trajectoryBuilder.trajectorySequenceRight;
            }
            else if (propLoc == TeamPropDetection.propLocation.CENTER) {
                finalTrajectory = trajectoryBuilder.trajectorySequenceCenter;
            }
            else {
                finalTrajectory = trajectoryBuilder.trajectorySequenceLeft;
            }
        }
        else {
            if (propLoc == TeamPropDetection.propLocation.RIGHT) {
                finalTrajectory = trajectoryBuilder.trajectorySequenceRight;
            } else if (propLoc == TeamPropDetection.propLocation.CENTER) {
                finalTrajectory = trajectoryBuilder.trajectorySequenceCenter;
            } else {
                finalTrajectory = trajectoryBuilder.trajectorySequenceLeft;
            }
        }

        drive.setPoseEstimate(c.startPose); // !!!!!

        waitForStart();

        drive.followTrajectorySequence(finalTrajectory.get(0));

        if(config.side == SIDE.AUDIENCE){
            intakeFromStack(finalTrajectory.get(1));
            drive.followTrajectorySequence(finalTrajectory.get(2));
            dropOffPixelsFromCycle(finalTrajectory.get(3), apriltags);
        }
        else{
            setPoseUsingATags(apriltags);
            robot.outTakeSetClawYawVertical.runAsync();
            dropOffPixelsFromBackdropPurple(finalTrajectory.get(1), apriltags);
        }
        drive.followTrajectorySequence(finalTrajectory.get((finalTrajectory.size())-1));

        AutoDataStorage.currentPose = drive.getPoseEstimate();
        AutoDataStorage.comingFromAutonomous = true;


        while(!isStopRequested()){
            // wait
        }

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

    private void dropOffPixelsFromBackdropPurple(TrajectorySequence goToBackdrop, AprilTagPoseDetection apriltags) {
        //robot.outTakeSetClawYawVertical.runAsync();
        drive.followTrajectorySequence(goToBackdrop);

        robot.wait(500, TimeUnit.MILLISECONDS);

        robot.openClaw.run();

        robot.wait(250, TimeUnit.MILLISECONDS);

        setPoseUsingATags(apriltags);

        robot.resetOutTake.runAsync();
    }

    private void dropOffPixelsFromCycle(TrajectorySequence goToBackdrop, AprilTagPoseDetection apriltags) {
        //robot.outTakeSetClawYawVertical.runAsync();
        setPoseUsingATags(apriltags);

        drive.followTrajectorySequence(goToBackdrop);

        robot.wait(500, TimeUnit.MILLISECONDS);

        robot.openClaw.run();

        robot.wait(250, TimeUnit.MILLISECONDS);

        robot.resetOutTake.runAsync();
    }

    private void intakeFromStack(TrajectorySequence intake) throws InterruptedException {
        robot.intake.setIntakeFlipperPosition(Intake.FlipperPosition.PIXEL5);


        robot.startIntakingPixels.run();
        robot.wait(500, TimeUnit.MILLISECONDS);
        drive.followTrajectorySequence(intake);

        robot.tryIntakeTwoPixels.run();


        while(!robot.intake.pixels.hasTwoPixels()) {
            robot.intake.pixels.update();
        }

        robot.holdPixels.run();
    }

    private void setPoseUsingATags(AprilTagPoseDetection apriltags) {
        int retries = 3;
        Pose2d correctedPose;
        do {
            robot.wait(200, TimeUnit.MILLISECONDS); // Makes sure the robot is still
            correctedPose = apriltags.getRobotPosFromTags();
            retries -= 1;
        } while(retries>0 && correctedPose.getX()==0);

        if(correctedPose.getX()!=0) {
            drive.setPoseEstimate(correctedPose);
        }
    }

}
