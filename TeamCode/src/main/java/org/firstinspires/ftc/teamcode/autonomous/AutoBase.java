package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Claw;
import org.firstinspires.ftc.teamcode.roadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadRunner.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.roadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.teamProp.TeamPropDetection;
import org.firstinspires.ftc.teamcode.tools.AutoDataStorage;
import org.firstinspires.ftc.teamcode.tools.Robot;
import org.firstinspires.ftc.teamcode.tools.Global;

import java.util.ArrayList;

//@Autonomous(name="Autonomous Base")
public abstract class AutoBase extends LinearOpMode {

    protected StandardTrackingWheelLocalizer myLocalizer;

    public class Coordinates{
        Boolean isBlueAlliance;
        Boolean isNearSide;

        //Blue near side
        Pose2d startPose = new Pose2d(12, 62, Math.toRadians(90.0));
        Pose2d leftTeamProp = new Pose2d(20, 38, Math.toRadians(90.0));
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
        Vector2d prepareFarDrop = new Vector2d(-37, 59);

        Vector2d backdropIntermediateFar = new Vector2d(18, 59);
        Vector2d intermediateDropFar = new Vector2d(35, 59);



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
                backdropIntermediateFar = flipVectorAcrossX(backdropIntermediateFar);

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

                    prepareFarDrop = flipVectorAcrossX(prepareFarDrop);
                }
            }
        }


        // Blue alliance to red alliance
        public Pose2d flipAcrossX(Pose2d pose){
            return new Pose2d(pose.getX(), -pose.getY(), (-pose.getHeading())%Math.toRadians(360));
        }

        public Pose2d flipAcrossXKeepHeading(Pose2d pose){ //needed for team prop and prepare far drop
            return new Pose2d(pose.getX(), -pose.getY(), pose.getHeading());
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
            //return new Pose2d(pose.getX()-48, -pose.getY(),(-pose.getHeading())%Math.toRadians(360));
        }

        public Pose2d flipTeamPropAcrossCenter(Pose2d pose){
            return new Pose2d(pose.getX()-48, -pose.getY(), pose.getHeading());
        }

//        public Pose2d flipBackDrop(Pose2d pose){
//            return new Pose2d(pose.getX(), pose.getY()-72, (-pose.getHeading())%Math.toRadians(360));
//        }

    }
    Coordinates c; //= new Coordinates(true, true); // change values later

    abstract void Setup();

    @Override
    public void runOpMode() throws InterruptedException {
        Setup();
        Global.telemetry = telemetry;
        Robot robot = new Robot(hardwareMap, gamepad1, gamepad2, true);

        TeamPropDetection teamPropDetection = new TeamPropDetection();
        teamPropDetection.Setup(hardwareMap, telemetry);
        telemetry.setMsTransmissionInterval(50);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        // hardware map for odometry encoders
        myLocalizer = new StandardTrackingWheelLocalizer(hardwareMap, null, null);
        // start location (coordinate)


        // Let's have at list 33% chance to pick it right if nothing works
        TeamPropDetection.propLocation propLoc = TeamPropDetection.propLocation.CENTER;

        Robot.lift.startLiftMotorWithEncoder(0.5);
        //Robot.lift.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //Robot.lift.liftMotor.setPower(0.5);
        while (Robot.lift.getCurrentLiftMotorPosition() < 15*0.95) {
            continue;
        }
        Robot.lift.stopLiftMotor();
        //Robot.lift.liftMotor.setPower(0);

        Robot.claw.setGripPosition(Claw.gripPositions.CLOSE_ONE_PIXEL);
        Robot.claw.setPitchPosition(Claw.pitchPositions.INTAKE);
        Robot.claw.setYawPosition(Claw.yawPositions.INTAKE);

        //Robot.clawGrip.setPosition(Robot.clawCloseOnePixel);
        //Robot.clawPitch.setPosition(Robot.clawPitchIntake);
        //Robot.clawYaw.setPosition(Robot.clawYawIntake);

        TrajectoryBuilder trajectoryBuilder = new TrajectoryBuilder(c, drive);
        ArrayList<TrajectorySequence> finalTrajectory;

        while (!isStarted() && !isStopRequested())
        {
            TeamPropDetection.propLocation currentPropLoc = teamPropDetection.GetPropLocation();
            if(currentPropLoc!=TeamPropDetection.propLocation.NULL) {
                propLoc = currentPropLoc;
                telemetry.addLine("Detected:" + propLoc);
                telemetry.update();
            }
        }

        myLocalizer.setPoseEstimate(c.startPose);
        drive.setPoseEstimate(c.startPose); // !!!!!

        if (propLoc == TeamPropDetection.propLocation.LEFT) {
            finalTrajectory = trajectoryBuilder.trajectorySequenceLeft;
        }
        else if (propLoc == TeamPropDetection.propLocation.CENTER) {
            finalTrajectory = trajectoryBuilder.trajectorySequenceCenter;
        }
        else {
            finalTrajectory = trajectoryBuilder.trajectorySequenceRight;
        }

        //robot.updateSync();


        //Raise lift so the pixel doesn't drag on the ground
        robot.autoOutTakeYellowLow.run();

        // Deposit purple pixel on spike mark
        drive.followTrajectorySequence(finalTrajectory.get(0));
        Robot.planeLauncher.store();


        // Raise lift more + angle the claw to outtake
        if(c.isNearSide) {
            robot.autoOutTakeYellow.runAsync();
        }
        else {
            ElapsedTime timer = new ElapsedTime();
            while(timer.seconds() < 9) {
                continue;
            }
        }
        // Position the robot in front of the backdrop
        drive.followTrajectorySequence(finalTrajectory.get(1));

        if(!c.isNearSide) {

            if(propLoc == TeamPropDetection.propLocation.CENTER)
                robot.autoOutTakeYellowHigh.runAsync();
            else {
                robot.autoOutTakeYellow.runAsync();

            }

        }

        // Go to the backdrop
        drive.followTrajectorySequence(finalTrajectory.get(2));

        // Drop yellow pixel
        robot.autoOpenClaw.run();

        // Park
        robot.exitingOutTakeToIdle.runAsync();
        drive.followTrajectorySequence(finalTrajectory.get(3));


        AutoDataStorage.currentPose = drive.getPoseEstimate();
        AutoDataStorage.comingFromAutonomous = true;

        waitForStart();

    }


}
