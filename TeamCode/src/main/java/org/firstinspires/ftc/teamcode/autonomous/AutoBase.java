package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
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
import org.firstinspires.ftc.teamcode.tools.Robot;
import org.firstinspires.ftc.teamcode.tools.Global;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.concurrent.TimeUnit;

//@Autonomous(name="Autonomous Base")
public abstract class AutoBase extends LinearOpMode {

    private Telemetry telemetryA;

    private Follower follower;

    private PathChain empty, purpleDrop, purpleToLeftSideStackSetup, goToBackdropCenterThroughCenterTruss, goToStackSetupThroughCenterTrussFromCenterBackdrop,goToStackSetupThroughCenterTrussFromLeftBackdrop, goToStackSetupThroughCenterTrussFromRightBackdrop, goToBackdropLeftThroughCenterTruss, goToBackdropRightThroughCenterTruss, backdropToLeftSideStack, park;
    private Deadline timer;
    Robot robot;

    protected StandardTrackingWheelLocalizer myLocalizer;
    SampleMecanumDrive drive;


    public static class Coordinates{
        Boolean isBlueAlliance;
        Boolean isNearSide;

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

        Pose2d stackCenter = new Pose2d(-56, 24, Math.toRadians(180));
        Pose2d stackCenterSetup = new Pose2d(stackCenter.getX()+10, stackCenter.getY(), stackCenter.getHeading());

        Pose2d stackLeft = new Pose2d(-57, 36, stackCenter.getHeading());
        Pose2d stackLeftSetup = new Pose2d(-48, 36, stackLeft.getHeading());


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

                stackCenter = flipAcrossX(stackCenter);
                stackCenterSetup = flipAcrossX(stackCenterSetup);
                stackLeft = flipAcrossX(stackLeft);
                stackLeftSetup = flipAcrossX(stackLeftSetup);
                stackRight = flipAcrossX(stackRight);
                stackRightSetup = flipAcrossX(stackRightSetup);

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


    private PathChain goToBackdropThroughCenterTruss(Pose2d backdropPosition) {
        return follower.pathBuilder()
                .addPath(new BezierLine(new Point(c.stackLeft), new Point(c.centerTruss)))
                .addParametricCallback(0.8, robot.outTake.getAsyncRunnable())
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(new BezierCurve(new Point(c.centerTruss), new Point(c.centerTrussToBackDropControlPoint), new Point(backdropPosition)))
                //.addParametricCallback(0.2, )
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
    private PathChain intakeFromStack(AutoBase.STACK_POSITIONS position){
        Pose2d setup = new Pose2d(), stack = new Pose2d();
        if (position == AutoBase.STACK_POSITIONS.LEFT){
            stack = c.stackLeft;
            setup = c.stackLeftSetup;
        }
        if (position == AutoBase.STACK_POSITIONS.CENTER){
            stack = c.stackCenter;
            setup = c.stackCenterSetup;
        }
        if (position == AutoBase.STACK_POSITIONS.RIGHT){
            stack = c.stackRight;
            setup = c.stackRightSetup;
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


        TrajectorySequence purpleDrop = drive.trajectorySequenceBuilder(c.startPose)
                .lineToLinearHeading(c.centerTeamProp)
                .build();

        TrajectorySequence stackSetup1 = drive.trajectorySequenceBuilder(purpleDrop.end())
                .forward(2)
                .splineToLinearHeading(c.stackLeftSetup, Math.toRadians(180))
                .build();

        TrajectorySequence intakeStack1 = drive.trajectorySequenceBuilder(stackSetup1.end())
                .lineToLinearHeading(c.stackLeft)
                .build();

        TrajectorySequence goToBackdrop1 = drive.trajectorySequenceBuilder(intakeStack1.end())
                .lineToLinearHeading(c.backdropCenter, SampleMecanumDrive.getVelocityConstraint(30, 30, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        TrajectorySequence stackSetup2 = drive.trajectorySequenceBuilder(goToBackdrop1.end())
                // Readjusts
                .lineToLinearHeading(new Pose2d(c.backdropCenter.getX()+0.1, c.backdropCenter.getY(), c.backdropCenter.getHeading()))
                .lineToLinearHeading(c.stackLeftSetup, SampleMecanumDrive.getVelocityConstraint(30, 30, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        TrajectorySequence intakeStack2 = drive.trajectorySequenceBuilder(stackSetup2.end())
                .lineToLinearHeading(new Pose2d(c.stackLeft.getX()-1, c.stackLeft.getY(), c.stackLeft.getHeading()))
                .build();

        TrajectorySequence goToBackdrop2 = drive.trajectorySequenceBuilder(intakeStack2.end())
                .lineToLinearHeading(new Pose2d(c.stackLeft.getX()+0.1, c.stackLeft.getY(), c.stackLeft.getHeading()))
                .lineToLinearHeading(c.backdropCenter, SampleMecanumDrive.getVelocityConstraint(30, 30, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        TrajectorySequence cycle2Dropoff = drive.trajectorySequenceBuilder(intakeStack2.end())
                .strafeLeft(7)
                .build();

        /*
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
*/
        telemetryA.addLine("Good to start, go for it.");
        telemetryA.update();
        Global.telemetry.speak("sharkbots is alive");


        // Let's have at list 33% chance to pick it right if nothing works
        TeamPropDetection.propLocation propLoc = TeamPropDetection.propLocation.CENTER;

//        TrajectoryBuilder trajectoryBuilder = new TrajectoryBuilder(c, drive);
//        ArrayList<TrajectorySequence> finalTrajectory;

        AprilTagPoseDetection apriltags = new AprilTagPoseDetection();
        apriltags.setup(c.isBlueAlliance, hardwareMap);

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
            TeamPropDetection.propLocation currentPropLoc = apriltags.GetPropLocation();
            if(currentPropLoc!=TeamPropDetection.propLocation.NULL) {
                propLoc = currentPropLoc;
                telemetry.addLine("Detected:" + propLoc);
                telemetry.update();
            }
        }

        apriltags.visionPortal.stopStreaming();
        apriltags.visionPortal.setProcessorEnabled(apriltags.aprilTag, true);
        apriltags.visionPortal.setProcessorEnabled(apriltags.teamPropDetectionPipeline, false);
        apriltags.visionPortal.resumeStreaming();


//        myLocalizer.setPoseEstimate(c.startPose);
//        drive.setPoseEstimate(c.startPose); // !!!!!
//
//        if (propLoc == TeamPropDetection.propLocation.LEFT) {
//            finalTrajectory = trajectoryBuilder.trajectorySequenceLeft;
//        }
//        else if (propLoc == TeamPropDetection.propLocation.CENTER) {
//            finalTrajectory = trajectoryBuilder.trajectorySequenceCenter;
//        }
//        else {

        waitForStart();

       TrajectorySequence untitled0 = drive.trajectorySequenceBuilder(new Pose2d(-36.00, 62.00, Math.toRadians(90.00)))
                .lineToLinearHeading(new Pose2d(-37.76, 31.43, Math.toRadians(0.00)))
               .build();


        TrajectorySequence purpleCenterAndStack = drive.trajectorySequenceBuilder(new Pose2d(12.00, 62.00, Math.toRadians(90.00)))
                .lineTo(new Vector2d(12.00, 34.00))
                .lineToLinearHeading(new Pose2d(12.00, 59.00, Math.toRadians(180.00)))
                .lineToLinearHeading(new Pose2d(-32.00, 60.00, Math.toRadians(198.00)))
                .lineToLinearHeading(c.stackRightSetup)
                .lineToLinearHeading(c.stackRight)
                .build();

        TrajectorySequence purpleLeftAndStack = drive.trajectorySequenceBuilder(new Pose2d(12.00, 62.00, Math.toRadians(90.00)))
                .lineTo(new Vector2d(12.00, 34.00))
                .lineTo(new Vector2d(10.00, 34.00))
                .lineToLinearHeading(new Pose2d(12.00, 59.00, Math.toRadians(180.00)))
                .lineToLinearHeading(new Pose2d(-32.00, 60.00, Math.toRadians(198.00)))
                .lineToLinearHeading(c.stackRightSetup)
                .lineToLinearHeading(c.stackRight)
                .build();

        TrajectorySequence purpleRightAndStack = drive.trajectorySequenceBuilder(new Pose2d(12.00, 62.00, Math.toRadians(90.00)))
                .lineToLinearHeading(new Pose2d(12.00, 34.00, Math.toRadians(0.00)))
                .lineTo(new Vector2d(14.00, 34.00))
                .lineToLinearHeading(new Pose2d(12.00, 59.00, Math.toRadians(180.00)))
                .lineToLinearHeading(new Pose2d(-32.00, 60.00, Math.toRadians(198.00)))
                .lineToLinearHeading(c.stackRightSetup)
                .lineToLinearHeading(c.stackRight)
                .build();




        drive.followTrajectorySequence(purpleCenterAndStack);









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
