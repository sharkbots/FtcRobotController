package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.google.ar.core.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.apache.commons.math3.geometry.euclidean.threed.PolyhedronsSet;
import org.firstinspires.ftc.teamcode.roadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadRunner.drive.StandardTrackingWheelLocalizer;
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

        Pose2d rightParkIntermediateBlueLeft = new Pose2d(42, 11.5, Math.toRadians(180.00));
        Pose2d rightParkFinalBlueLeft = new Pose2d(50, 11.5, Math.toRadians(180.00));
        Pose2d leftParkIntermediateRedRight = new Pose2d(42, -11.5, Math.toRadians(180.00));
        Pose2d leftParkFinalRedRight = new Pose2d(50, -11.5, Math.toRadians(180.00));


        // DBZ Right park on Red team
        Pose2d rightParkIntermediateRedRight = new Pose2d(42, -60, Math.toRadians(180.00));
        Pose2d rightParkFinalRedRight = new Pose2d(50, -60, Math.toRadians(180.00));




        //left backdrop
        Pose2d leftBackdropLeft = new Pose2d(50, 44, Math.toRadians(180.00));
        Pose2d leftBackdropCenter = new Pose2d(50, 36, Math.toRadians(180.00));
        Pose2d leftBackdropRight = new Pose2d(50, 30, Math.toRadians(180.00));

        // close side
        Pose2d leftBackdropIntermediateLeft = new Pose2d(30, 42, Math.toRadians(180.00));
        Pose2d leftBackdropIntermediateCenter = new Pose2d(35, 36, Math.toRadians(180.00));
        Pose2d leftBackdropIntermediateRight = new Pose2d(30, 30, Math.toRadians(180.00));


        //right backdrop
        Pose2d rightBackdropLeft = new Pose2d(50, -29, Math.toRadians(180.00));
        Pose2d rightBackdropCenter = new Pose2d(50, -36, Math.toRadians(180.00));
        Pose2d rightBackdropRight = new Pose2d(50, -44, Math.toRadians(180.00));


        Pose2d rightBackdropIntermediateLeft = new Pose2d(30, -30, Math.toRadians(180.00));
        Pose2d rightBackdropIntermediateCenter = new Pose2d(35, -36, Math.toRadians(180.00));
        Pose2d rightBackdropIntermediateRight = new Pose2d(30, -42, Math.toRadians(180.00));


        //Blue Left
        Pose2d preStartPoseBlueLeft = new Pose2d(9.5, 63, Math.toRadians(90)); //robot needs to strafe 2 inches to the actual start pose
        Pose2d startPoseBlueLeft = new Pose2d(14, 63, Math.toRadians(90));
        Pose2d rightTeamPropBlueLeft = new Pose2d(10.5, 32, Math.toRadians(0.00));
        Pose2d centerTeamPropBlueLeft = new Pose2d(12, 34.5, Math.toRadians(90.00));
        Pose2d leftTeamPropBlueLeft = new Pose2d(9.5, 28, Math.toRadians(180.00));

        //Blue right
        Pose2d preStartPoseBlueRight = new Pose2d(-9.5, 63, Math.toRadians(90));
        Pose2d startPoseBlueRight = new Pose2d(-14, 61, Math.toRadians(90));
        Pose2d leftTeamPropBlueRight = new Pose2d(-11, 28, Math.toRadians(160));
        Pose2d centerTeamPropBlueRight = new Pose2d(-12, 35, Math.toRadians(90.00));
        Pose2d rightTeamPropBlueRight = new Pose2d(-11.5, 39, Math.toRadians(0));

        //Red left
        Pose2d preStartPoseRedLeft = new Pose2d(-9.5, -63, Math.toRadians(270));
        Pose2d startPoseRedLeft = new Pose2d(-13, -63, Math.toRadians(270));
        Pose2d rightTeamPropRedLeft = new Pose2d(-10, -32, Math.toRadians(180.00));
        Pose2d centerTeamPropRedLeft = new Pose2d(-12, -34.5, Math.toRadians(270));
        Pose2d leftTeamPropRedLeft = new Pose2d(-10.5, -30, Math.toRadians(0));

        //Red right
        Pose2d preStartPoseRedRight = new Pose2d(9.5, -63, Math.toRadians(270));
        Pose2d startPoseRedRight = new Pose2d(15.5, -60, Math.toRadians(270));
        Pose2d rightTeamPropRedRight = new Pose2d(9.5, -28, Math.toRadians(180.00));
        Pose2d centerTeamPropRedRight = new Pose2d(12, -35, Math.toRadians(270));
        Pose2d leftTeamPropRedRight = new Pose2d(10, -31, Math.toRadians(350));


        public Coordinates(Boolean BlueAlliance, Boolean CloseSide) {
            this.BlueAlliance = BlueAlliance;
            this.CloseSide = CloseSide;

            // Default is blue alliance close side

            // Blue alliance far side
            if(BlueAlliance && !CloseSide){

            }

            // Red alliance
            if (!BlueAlliance){

                // Close side
                if (CloseSide){

                }

                // Far side
                if (!CloseSide){

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
    Coordinates c = new Coordinates(true, true);
    static final double SLOWERVELOCITY = 15;
    static final double SLOWERANGULARVELOCITY = 2.5;

    public abstract void runAutonomous(Robot robot, SampleMecanumDrive drive, TeamPropDetection.propLocation propLoc);

    @Override
    public void runOpMode() throws InterruptedException {
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


        waitForStart();

        runAutonomous(robot, drive, propLoc);
        AutoDataStorage.currentPose = drive.getPoseEstimate();
        AutoDataStorage.comingFromAutonomous = true;
    }


}
