package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.roadRunner.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.roadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadRunner.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.roadRunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.teamProp.TeamPropDetection;
import org.firstinspires.ftc.teamcode.tools.AutoDataStorage;
import org.firstinspires.ftc.teamcode.tools.Robot;
import org.firstinspires.ftc.teamcode.tools.Global;

//@Autonomous(name="Autonomous Base")
public abstract class AutoBase extends LinearOpMode {

    protected StandardTrackingWheelLocalizer myLocalizer;

    public class Coordinates{
        final Boolean BlueAlliance;
        final Boolean CloseSide;

        // Blue alliance parking
        Pose2d parkIntermediate = new Pose2d(42, 11.5, Math.toRadians(180.00));
        Pose2d parkFinal = new Pose2d(50, 11.5, Math.toRadians(180.00));
        /*Pose2d leftParkIntermediateRedRight = new Pose2d(42, -11.5, Math.toRadians(180.00));
        Pose2d leftParkFinalRedRight = new Pose2d(50, -11.5, Math.toRadians(180.00));*/


        // DBZ Right park on Red team
        Pose2d rightParkIntermediateRedRight = new Pose2d(42, -60, Math.toRadians(180.00));
        Pose2d rightParkFinalRedRight = new Pose2d(50, -60, Math.toRadians(180.00));




        //Blue backdrop
        Pose2d backdropLeft = new Pose2d(50, 44, Math.toRadians(180.00));
        Pose2d backdropCenter = new Pose2d(50, 36, Math.toRadians(180.00));
        Pose2d backdropRight = new Pose2d(50, 30, Math.toRadians(180.00));

        // close side
        Pose2d backdropIntermediateLeft = new Pose2d(30, 42, Math.toRadians(180.00));
        Pose2d backdropIntermediateCenter = new Pose2d(35, 36, Math.toRadians(180.00));
        Pose2d backdropIntermediateRight = new Pose2d(30, 30, Math.toRadians(180.00));



        /*// red backdrop
        Pose2d rightBackdropLeft = new Pose2d(50, -29, Math.toRadians(180.00));
        Pose2d rightBackdropCenter = new Pose2d(50, -36, Math.toRadians(180.00));
        Pose2d rightBackdropRight = new Pose2d(50, -44, Math.toRadians(180.00));


        // Close side approach to backdrop
        Pose2d rightBackdropIntermediateLeft = new Pose2d(30, -30, Math.toRadians(180.00));
        Pose2d rightBackdropIntermediateCenter = new Pose2d(35, -36, Math.toRadians(180.00));
        Pose2d rightBackdropIntermediateRight = new Pose2d(30, -42, Math.toRadians(180.00));*/


        // Purple spike mark locations
        //Blue close side
        Pose2d preStartPose = new Pose2d(9.5, 63, Math.toRadians(90)); //robot needs to strafe 2 inches to the actual start pose
        Pose2d startPose = new Pose2d(14, 63, Math.toRadians(90));
        Pose2d rightTeamProp = new Pose2d(10.5, 32, Math.toRadians(0.00));
        Pose2d centerTeamProp = new Pose2d(12, 34.5, Math.toRadians(90.00));
        Pose2d leftTeamProp = new Pose2d(9.5, 28, Math.toRadians(180.00));

        /*//Blue far side
        Pose2d preStartPoseBlueRight = new Pose2d(-9.5, 63, Math.toRadians(90));
        Pose2d startPoseBlueRight = new Pose2d(-14, 61, Math.toRadians(90));
        Pose2d leftTeamPropBlueRight = new Pose2d(-11, 28, Math.toRadians(160));
        Pose2d centerTeamPropBlueRight = new Pose2d(-12, 35, Math.toRadians(90.00));
        Pose2d rightTeamPropBlueRight = new Pose2d(-11.5, 39, Math.toRadians(0));

        //Red far side
        Pose2d preStartPoseRedLeft = new Pose2d(-9.5, -63, Math.toRadians(270));
        Pose2d startPoseRedLeft = new Pose2d(-13, -63, Math.toRadians(270));
        Pose2d rightTeamPropRedLeft = new Pose2d(-10, -32, Math.toRadians(180.00));
        Pose2d centerTeamPropRedLeft = new Pose2d(-12, -34.5, Math.toRadians(270));
        Pose2d leftTeamPropRedLeft = new Pose2d(-10.5, -30, Math.toRadians(0));

        //Red close side
        Pose2d preStartPoseRedRight = new Pose2d(9.5, -63, Math.toRadians(270));
        Pose2d startPoseRedRight = new Pose2d(15.5, -60, Math.toRadians(270));
        Pose2d rightTeamPropRedRight = new Pose2d(9.5, -28, Math.toRadians(180.00));
        Pose2d centerTeamPropRedRight = new Pose2d(12, -35, Math.toRadians(270));
        Pose2d leftTeamPropRedRight = new Pose2d(10, -31, Math.toRadians(350));*/


        public Coordinates(Boolean BlueAlliance, Boolean CloseSide) {
            this.BlueAlliance = BlueAlliance;
            this.CloseSide = CloseSide;
            AutoDataStorage.redSide = false;

            // Default is blue alliance close side

            // Blue alliance far side
            if(BlueAlliance && !CloseSide){
                preStartPose = flipCloseToFarSide(preStartPose);
                startPose = flipCloseToFarSide(startPose);
                rightTeamProp = flipCloseToFarSide(rightTeamProp);
                centerTeamProp = flipCloseToFarSide(centerTeamProp);
                leftTeamProp = flipCloseToFarSide(leftTeamProp);
            }

            // Red alliance
            if (!BlueAlliance){
                // Red backdrop
                backdropLeft = flipBackDrop(backdropLeft);
                backdropCenter = flipBackDrop(backdropCenter);
                backdropRight = flipBackDrop(backdropRight);
                
                parkIntermediate = flipAcrossX(parkIntermediate);
                parkFinal = flipAcrossX(parkFinal);

                AutoDataStorage.redSide = true;

                // Close side
                if (CloseSide){
                    backdropIntermediateLeft = flipBackDrop(backdropIntermediateLeft);
                    backdropIntermediateCenter = flipBackDrop(backdropIntermediateCenter);
                    backdropIntermediateRight = flipBackDrop(backdropIntermediateRight);

                    preStartPose = flipAcrossX(preStartPose);
                    startPose = flipAcrossX(startPose);
                    rightTeamProp = flipAcrossX(rightTeamProp);
                    centerTeamProp = flipAcrossX(centerTeamProp);
                    leftTeamProp = flipAcrossX(leftTeamProp);
                }

                // Far side
                if (!CloseSide){
                    preStartPose = flipAcrossCenter(preStartPose);
                    startPose = flipAcrossCenter(startPose);
                    rightTeamProp = flipAcrossCenter(rightTeamProp);
                    centerTeamProp = flipAcrossCenter(centerTeamProp);
                    leftTeamProp = flipAcrossCenter(leftTeamProp);
                }
            }
        }


        // Blue alliance to red alliance
        public Pose2d flipAcrossX(Pose2d pose){
            return new Pose2d(pose.getX(), -pose.getY(), (-pose.getHeading())%360);
        }

        // Close side to far side
        public Pose2d flipCloseToFarSide(Pose2d pose){
            return new Pose2d(pose.getX()-48, pose.getY(), pose.getHeading());
        }

        public Pose2d flipAcrossCenter(Pose2d pose) {
            return flipCloseToFarSide(flipAcrossX(pose));
        }

        public Pose2d flipBackDrop(Pose2d pose){
            return new Pose2d(pose.getX(), pose.getY()-72, (-pose.getHeading())%360);
        }
    }
    Coordinates c; //= new Coordinates(true, true); // change values later
    static final double SLOWERVELOCITY = 15;
    static final double SLOWERANGULARVELOCITY = 2.5;

    public abstract void runAutonomous(Robot robot, SampleMecanumDrive drive, TeamPropDetection.propLocation propLoc);

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

        Robot.lift.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Robot.lift.liftMotor.setPower(0.5);
        while (Robot.lift.liftMotor.getCurrentPosition() < 15*0.95) {
            continue;
        }
        Robot.lift.liftMotor.setPower(0);

        Robot.clawGrip.setPosition(Robot.clawCloseOnePixel);
        Robot.clawPitch.setPosition(Robot.clawPitchIntake);
        Robot.clawYaw.setPosition(Robot.clawYawIntake);


        while (!isStarted() && !isStopRequested())
        {
            TeamPropDetection.propLocation currentPropLoc = teamPropDetection.GetPropLocation();
            if(currentPropLoc!=TeamPropDetection.propLocation.NULL) {
                propLoc = currentPropLoc;
                telemetry.addLine("Detected:" + propLoc);
                telemetry.update();
            }
        }








        myLocalizer.setPoseEstimate(c.preStartPose);
        drive.setPoseEstimate(c.preStartPose); // !!!!!

        Pose2d teamPropCoordinate;
        Pose2d backdropCoordinate;
        if (propLoc == TeamPropDetection.propLocation.LEFT) {
            teamPropCoordinate = c.leftTeamProp;
            backdropCoordinate = c.backdropLeft;
        }
        else if (propLoc == TeamPropDetection.propLocation.CENTER) {
            teamPropCoordinate = c.centerTeamProp;
            backdropCoordinate = c.backdropCenter;
        }
        else {
            teamPropCoordinate = c.rightTeamProp;
            backdropCoordinate = c.backdropRight;
        }

        // hardware map to get motors and sensors
        TrajectorySequence purpleDrop = drive.trajectorySequenceBuilder(c.preStartPose)
                //.lineTo(c.leftTeamProp)
                //.lineTo(c.centerTeamProp)
                .lineToLinearHeading(c.startPose)
                .lineToLinearHeading(teamPropCoordinate)
                .build();

        TrajectorySequence setupForBackdrop = drive.trajectorySequenceBuilder(purpleDrop.end())
                .back(3.5)
                .forward(3)
                .lineTo(new Vector2d(-32, 11))
                .lineTo(new Vector2d(-50, 11))
                .lineTo(new Vector2d(-50, 35))
                .build();

        TrajectorySequence goToBackdrop = drive.trajectorySequenceBuilder(setupForBackdrop.end())
                .lineToLinearHeading(c.backdropIntermediateCenter)
                .lineToLinearHeading(backdropCoordinate, SampleMecanumDrive.getVelocityConstraint(SLOWERVELOCITY, SLOWERANGULARVELOCITY, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        TrajectorySequence parkRight = drive.trajectorySequenceBuilder(goToBackdrop.end())
                .forward(8)
                .lineToLinearHeading(c.parkIntermediate)
                .lineToLinearHeading(c.parkFinal)
                .build();



        robot.closeClaw = true;
        robot.updateSync();
        // Test propLoc here
        drive.followTrajectorySequence(purpleDrop);

        robot.outtakePixels = true;
        robot.updateSync();
        drive.followTrajectorySequence(goToBackdrop);
        robot.closeClaw = false;
        robot.updateSync();
        drive.followTrajectorySequence(parkRight);











        waitForStart();

        //runAutonomous(robot, drive, propLoc);
        AutoDataStorage.currentPose = drive.getPoseEstimate();
        AutoDataStorage.comingFromAutonomous = true;
    }


}
