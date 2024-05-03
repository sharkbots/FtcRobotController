package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.aprilTags.AprilTagDetection;
import org.firstinspires.ftc.teamcode.tools.AutoDataStorage;
import org.firstinspires.ftc.teamcode.tools.SetDriveMotors;
import org.firstinspires.ftc.teamcode.tools.Global;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleOpDrive", group = "Testing")
public class TeleopDrive extends LinearOpMode {
    private SetDriveMotors driveMotors;

    private boolean isLiftReset = false;

    Robot robot;
    AprilTagDetection aprilTagDetection;
    Pose2d startPose;

    public void Setup(){
        Global.telemetry = telemetry;
        driveMotors = new SetDriveMotors(hardwareMap);

        robot = new Robot(hardwareMap, gamepad1, gamepad2);

        aprilTagDetection = new AprilTagDetection();
        aprilTagDetection.Setup(hardwareMap, telemetry);

        Robot.claw.setPitchPosition(Claw.pitchPositions.INTAKE);
        Robot.claw.setYawPosition(Claw.yawPositions.INTAKE);
        Robot.claw.setGripPosition(Claw.gripPositions.OPEN);
        Robot.planeLauncher.storePlane();
        Robot.intake.setIntakeFlipperPosition(Intake.FlipperPosition.UP);
        /*Robot.clawPitch.setPosition(Robot.clawPitchIntake); // clawPitchIntake
        Robot.clawYaw.setPosition(Robot.clawYawIntake);
        Robot.clawGrip.setPosition(Robot.clawOpen);
        Robot.planeAngle.setPosition(Robot.planeAngleStore);*/
        sleep(1000);

        while(!isStarted() && !isStopRequested()){
            if(!isLiftReset){
                Robot.lift.startLiftMotorWithEncoder(-1);
                //Robot.lift.liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                //Robot.lift.liftMotor.setPower(-1);
                if(Robot.lift.liftTouchDownPressed()){
                    Robot.lift.resetLiftMotorEncoder();
                    /*Robot.lift.liftMotor.setPower(0);
                    Robot.lift.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);*/
                    isLiftReset = true;
                }
        }
        }
        AutoDataStorage.comingFromAutonomous = false;

    }

    @Override
    public void runOpMode() throws InterruptedException {
        Setup();
        //Global.telemetry.speak("SHARKBOTS SHARKBOTS SHARKBOTS");
        waitForStart();
        while(opModeIsActive()){

            if(!isLiftReset) {
                Robot.lift.startLiftMotorWithNoEncoder(-1);
                //Robot.lift.liftMotor.setPower(-1);
                if (Robot.lift.liftTouchDownPressed()) {
                    Robot.lift.stopLiftMotor();
                    //Robot.lift.liftMotor.setPower(0);
                    isLiftReset = true;
                }
            }

            //telemetry.addLine("Pose2d: x: " + driveMotors.pluh().getX() + " y: " + driveMotors.pluh().getY());

            Global.telemetry.update();
            double horizontal = gamepad1.left_stick_x;
            double vertical = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            boolean goFast = gamepad1.left_bumper;
            boolean emergencyBrakeOverride = gamepad1.right_bumper;
            boolean switchDriveMode = gamepad1.b;
            boolean alignToCardinalPoint = gamepad1.a;
            boolean resetHeading = gamepad1.y;

            double distanceToWall = 0;
            if (!emergencyBrakeOverride){
                // AprilTag detection of positions if costly
                // Put calculation within if test so it s performed when needed only
                // thus no calc is in emergency override
                distanceToWall = aprilTagDetection.GetDistanceAwayFromTheBackdrop();
            }



            driveMotors.driveCommands(horizontal, vertical, turn, goFast, distanceToWall, switchDriveMode, alignToCardinalPoint, resetHeading);
            driveMotors.update();

            robot.update();

            if(robot.currentState()==robot.outTakingPixels) {
                Robot.claw.update();
            }

        }
    }

}