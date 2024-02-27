package org.firstinspires.ftc.teamcode.tools;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.roadRunner.drive.SampleMecanumDrive;

@TeleOp
@Disabled
public class SetDriveMotors extends OpMode {

    public final double DEADZONE_MIN_Y = 0.1;
    public final double DEADZONE_MIN_X = 0.25;
    public final double AUTOBRAKE_DISTANCE = 0.5;
    private final DeadzoneSquare horizontalFastDeadzone;
    private final DeadzoneSquare verticalFastDeadzone;
    private final DeadzoneSquare horizontalSlowDeadzone;
    private final DeadzoneSquare verticalSlowDeadzone;
    private final DcMotor backLeftMotor;
    private final DcMotor frontLeftMotor;
    private final DcMotor backRightMotor;
    private final DcMotor frontRightMotor;
    //private final IMU imu;
    private final SampleMecanumDrive drive;

    protected enum DriveMode{
        FIELD_CENTRIC,
        ROBOT_CENTRIC
    }

    private DriveMode driveMode = DriveMode.FIELD_CENTRIC;

    static final double BACKDROP_APPROACH_SPEED = -0.25;

    //map the motors and run the op mode
    public SetDriveMotors(HardwareMap hardwareMap) {
        // Initialize SampleMecanumDrive
        drive = new SampleMecanumDrive(hardwareMap);

        // Retrieve our pose from the PoseStorage.currentPose static field
        // See AutoTransferPose.java for further details
        if (AutoDataStorage.comingFromAutonomous){
            drive.setPoseEstimate(AutoDataStorage.currentPose);
        }

        else {
            //Pose2d rightParkFinalBlueLeft = new Pose2d(-11.5, 50, Math.toRadians(270.00));
            drive.setPoseEstimate(new Pose2d(-11.5, 50, Math.toRadians(270.00)));
        }

        // Retrieve the IMU from the hardware map
        /*imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.FORWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);
        imu.resetYaw();*/
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");

        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Reverse right side motor directions
        // This may need to be flipped to the left side depending on your motor rotation direction
        //frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        double deadzone = 0.1;

        horizontalFastDeadzone = new DeadzoneSquare(deadzone, DEADZONE_MIN_X, 1.5); //10% joystick deadzone. 0.25 is minimum power to strafe. 0.75 is the max power.
        verticalFastDeadzone = new DeadzoneSquare(deadzone, DEADZONE_MIN_Y, 1.5);
        horizontalSlowDeadzone = new DeadzoneSquare(deadzone, DEADZONE_MIN_X, 0.6);
        verticalSlowDeadzone = new DeadzoneSquare(deadzone, DEADZONE_MIN_Y, 0.5);
    }
    public void driveCommands(double horizontal, double vertical, double turn, boolean goFast, double distanceToWallMeters, boolean switchDriveMode, boolean alignToCardinalPoint, boolean resetHeading) {

        // Read pose
        Pose2d poseEstimate = drive.getPoseEstimate();

        if (resetHeading){
            drive.setPoseEstimate(new Pose2d(poseEstimate.getY(), Math.toRadians(270)));
        }

        //deadzones
        if (goFast && !isAutoBrake(distanceToWallMeters, poseEstimate)) {
            horizontal = horizontalFastDeadzone.computePower(horizontal);
            vertical = verticalFastDeadzone.computePower(vertical);
            turn *=1.5;
        } else {
            horizontal = horizontalSlowDeadzone.computePower(horizontal);
            vertical = verticalSlowDeadzone.computePower(vertical);
            turn *= 0.6;
        }

        if(switchDriveMode) {
            driveMode = driveMode==DriveMode.ROBOT_CENTRIC? DriveMode.FIELD_CENTRIC : DriveMode.ROBOT_CENTRIC;
        }

        if(driveMode == DriveMode.ROBOT_CENTRIC){
            //Driver assistance: takes over if too close to wall
            if (isAutoBrake(distanceToWallMeters, poseEstimate)){
                if (vertical < BACKDROP_APPROACH_SPEED) {
                    vertical = BACKDROP_APPROACH_SPEED;
                }
            }
            double rotX = horizontal;
            double rotY = vertical;

            double rotationalCorrection = 1.0; // original value of code on site was 1.1

            rotX = rotX * rotationalCorrection;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(turn), 1);

            // uses the values calculated via the imu's yaw to create accurate movement
            double frontLeftPower = (rotY + rotX + turn) / denominator;
            double backLeftPower = (rotY - rotX + turn) / denominator;
            double frontRightPower = (rotY - rotX - turn) / denominator;
            double backRightPower = (rotY + rotX - turn) / denominator;

            // Set motor power based on the calculated values
            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);
        }
        else { // DriveMode.FIELD_CENTRIC

            Global.telemetry.addData("getX: ", poseEstimate.getX());
            Global.telemetry.addData("getY: ", poseEstimate.getY());
            Global.telemetry.addData("Distance to wall: ", distanceToWallMeters);

            if (isAutoBrake(distanceToWallMeters, poseEstimate)){
                if (AutoDataStorage.redSide) {
                    horizontal = Math.min(-BACKDROP_APPROACH_SPEED,  horizontal);
                }
                else{
                    horizontal = Math.max(BACKDROP_APPROACH_SPEED,  horizontal);
                }
            }

            // Create a vector from the gamepad x/y inputs
            // Then, rotate that vector by the inverse of that heading
            Vector2d input = new Vector2d(
                    vertical,
                    -horizontal // Note: if the robot has a flipped left / right direction, make this negative
            ).rotated(-poseEstimate.getHeading());

            // If redSide is true, adjust the heading by 180 degrees
            if (AutoDataStorage.redSide) {
                input = input.rotated(Math.toRadians(90)); // Rotate by 180 degrees
            }
            else {
                input = input.rotated(Math.toRadians(-90)); // Rotate by 180 degrees
            }

            if (alignToCardinalPoint){
                turn = getAngleToCardinalPoint();
            }

            // Pass in the rotated input + right stick value for rotation
            // Rotation is not part of the rotated input thus must be passed in separately
            drive.setWeightedDrivePower(
                    new Pose2d(
                            input.getX(),
                            input.getY(),
                            -turn// Note: if the robot has a flipped turn direction, make this negative
                    )
            );
        }
    }

    private boolean isAutoBrake(double distanceToWallMeters, Pose2d poseEstimate) {
        return distanceToWallMeters != 0 &&
                distanceToWallMeters < AUTOBRAKE_DISTANCE &&
                poseEstimate.getX() > 35 &&
                ((poseEstimate.getY() > -57 && poseEstimate.getY() < -15) ||
                        (poseEstimate.getY() < 57 && poseEstimate.getY() > 15));
    }

    public void update() {
            drive.update();
    }
    public double getAngleToCardinalPoint()  {
        Pose2d poseEstimate = drive.getPoseEstimate();

        // Normalize the heading to the range [0, 2π)
        double currentHeading = Angle.norm(poseEstimate.getHeading());

        // Calculate the closest cardinal direction in radians
        double closestCardinalRad = Math.round(currentHeading / (Math.PI / 2)) * (Math.PI / 2);

        // Ensure the closestCardinalRad value is within [0, 2π)
        closestCardinalRad = Angle.norm(closestCardinalRad);

        Global.telemetry.addData("closestCard: ", Math.toDegrees(closestCardinalRad));
        Global.telemetry.addData("currHeading: ", Math.toDegrees(currentHeading));
        // Calculate the difference between current and closest cardinal direction

        // Normalize the difference to be within (-π, π]
        return Angle.normDelta(currentHeading - closestCardinalRad);
    }


    @Override
    public void init() {

    }

    @Override
    public void loop() {

    }
}