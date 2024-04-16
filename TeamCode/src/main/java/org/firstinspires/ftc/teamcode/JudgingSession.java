package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.aprilTags.AprilTagDetection;
import org.firstinspires.ftc.teamcode.tools.AutoDataStorage;
import org.firstinspires.ftc.teamcode.tools.Global;
import org.firstinspires.ftc.teamcode.tools.JudgingRobot;
import org.firstinspires.ftc.teamcode.tools.Robot;
import org.firstinspires.ftc.teamcode.tools.SetDriveMotors;

@TeleOp(name = "Judging Session", group = "Testing")
public class JudgingSession extends LinearOpMode {
    private SetDriveMotors driveMotors;

    private boolean isLiftReset = false;

    JudgingRobot robot;
    AprilTagDetection aprilTagDetection;
    Pose2d startPose;

    public void Setup(){
        Global.telemetry = telemetry;
        driveMotors = new SetDriveMotors(hardwareMap);

        robot = new JudgingRobot(hardwareMap, gamepad1, gamepad2);

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

            Global.telemetry.update();

            robot.update();
            Robot.lift.update();
            Robot.claw.update();

        }
    }



}