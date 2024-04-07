/* Copyright (c) 2023 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.aprilTags;

import static org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase.getCenterStageTagLibrary;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;


import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.tools.Global;
import org.firstinspires.ftc.teamcode.tools.math.Pose2dGeometry;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


import java.util.List;
import java.util.concurrent.TimeUnit;

/*
 * This OpMode illustrates using a camera to locate and drive towards a specific AprilTag.
 * The code assumes a Holonomic (Mecanum or X Drive) Robot.
 *
 * For an introduction to AprilTags, see the ftc-docs link below:
 * https://ftc-docs.firstinspires.org/en/latest/apriltag/vision_portal/apriltag_intro/apriltag-intro.html
 *
 * When an AprilTag in the TagLibrary is detected, the SDK provides location and orientation of the tag, relative to the camera.
 * This information is provided in the "ftcPose" member of the returned "detection", and is explained in the ftc-docs page linked below.
 * https://ftc-docs.firstinspires.org/apriltag-detection-values
 *
 * The drive goal is to rotate to keep the Tag centered in the camera, while strafing to be directly in front of the tag, and
 * driving towards the tag to achieve the desired distance.
 * To reduce any motion blur (which will interrupt the detection process) the Camera exposure is reduced to a very low value (5mS)
 * You can determine the best Exposure and Gain values by using the ConceptAprilTagOptimizeExposure OpMode in this Samples folder.
 *
 * The code assumes a Robot Configuration with motors named: leftfront_drive and rightfront_drive, leftback_drive and rightback_drive.
 * The motor directions must be set so a positive power goes forward on all wheels.
 * This sample assumes that the current game AprilTag Library (usually for the current season) is being loaded by default,
 * so you should choose to approach a valid tag ID (usually starting at 0)
 *
 * Under manual control, the left stick will move forward/back & left/right.  The right stick will rotate the robot.
 * Manually drive the robot until it displays Target data on the Driver Station.
 *
 * Press and hold the *Left Bumper* to enable the automatic "Drive to target" mode.
 * Release the Left Bumper to return to manual driving mode.
 *
 * Under "Drive To Target" mode, the robot has three goals:
 * 1) Turn the robot to always keep the Tag centered on the camera frame. (Use the Target Bearing to turn the robot.)
 * 2) Strafe the robot towards the centerline of the Tag, so it approaches directly in front  of the tag.  (Use the Target Yaw to strafe the robot)
 * 3) Drive towards the Tag to get to the desired distance.  (Use Tag Range to drive the robot forward/backward)
 *
 * Use DESIRED_DISTANCE to set how close you want the robot to get to the target.
 * Speed and Turn sensitivity can be adjusted using the SPEED_GAIN, STRAFE_GAIN and TURN_GAIN constants.
 *
 * Use Android Studio to Copy this Class, and Paste it into the TeamCode/src/main/java/org/firstinspires/ftc/teamcode folder.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 *
 */


public class AprilTagPoseDetection
{
    // We can define a coordinate system with the robot center as the origin.
    // Coordinates of the camera pose on that coordinate system.
    private final double CAMERA_RELATIVE_X_FROM_ROBOT_CENTER = -8.25;
    private final double CAMERA_RELATIVE_Y_FROM_ROBOT_CENTER = 0.0;

    private final double FIELD_BACKDROP_X_OFFSET = 2.0; //Backdrop can be setup differently than game manual
    private final double FIELD_BACKDROP_Y_OFFSET = 0.0; // Check World fields to set offset if any

    private final double CAMERA_RELATIVE_HEADING = Math.toRadians(180);
    private final Pose2d CAMERA_POS_RELATIVE_TO_ROBOT = new Pose2d(CAMERA_RELATIVE_X_FROM_ROBOT_CENTER, CAMERA_RELATIVE_Y_FROM_ROBOT_CENTER, CAMERA_RELATIVE_HEADING);
    private final double CAMERA_DISTANCE_FROM_ROBOT_CENTER = CAMERA_POS_RELATIVE_TO_ROBOT.vec().norm();
    private final double CAMERA_TO_ROBOT_BEARING = Math.atan(CAMERA_RELATIVE_Y_FROM_ROBOT_CENTER/CAMERA_RELATIVE_X_FROM_ROBOT_CENTER);

//
    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera

    private static final boolean DETECT_ALL = false;

    private static final int[] BLUE_DESIRED_TAG_IDS = {1, 2, 3}; // Blue backdrop ids
    private static final int[] RED_DESIRED_TAG_IDS = {4, 5, 6}; // Red backdrop ids
    private boolean IS_BLUE_SIDE = true;

    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.

    private final AprilTagLibrary centerStageTags = getCenterStageTagLibrary();

    

    private Pose2d getRobotPosFromTags() {
        // Step through the list of detected tags and look for a matching tag
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        Vector2d sumRobotVector = new Vector2d(0, 0);
        double sumRobotHeading = 0.0;
        int nbDetectedTags = 0;

        for (AprilTagDetection detection : currentDetections) {
            // Look to see if we have size info on this tag.
            if (detection.metadata == null) {
                // This tag is NOT in the library, so we don't have enough information to track to it.
                Global.telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
                continue;
            }

            //  Check to see if we want to track towards this tag.
            if (isDesiredTag(detection.id) || DETECT_ALL) {
                // Yes, we want to use this tag.
                Pose2d robotPose = getRobotPoseFromTag(detection);
                nbDetectedTags += 1;
                sumRobotVector = sumRobotVector.plus(robotPose.vec());
                sumRobotHeading += robotPose.getHeading();
            } else {
                // This tag is in the library, but we do not want to track it right now.
                Global.telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
            }

        }

        // Tell the driver what we see, and what to do.
        if (nbDetectedTags>0) {

            Global.telemetry.update();
            return new Pose2d(sumRobotVector.div(nbDetectedTags), sumRobotHeading / nbDetectedTags);

        } else {

            Global.telemetry.addData("\n>","No tags found\n");
            Global.telemetry.update();

            return new Pose2d(sumRobotVector, sumRobotHeading);
        }
    }

    /**
     * Initialize the AprilTag processor.
     */
    private void setup(Boolean isBlueSide, HardwareMap hardwareMap) {
        this.IS_BLUE_SIDE = isBlueSide;

        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(2);

        // Create the vision portal by using a builder.
        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .setLiveViewContainerId(hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName()))
                    .setCameraResolution(new Size(800, 600))
                    .enableLiveView(true)
                    .setAutoStopLiveView(true)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }

        if (USE_WEBCAM)
            setManualExposure(6, 250);  // Use low exposure time to reduce motion blur

        // Wait for driver to press start
        Global.telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
        Global.telemetry.addData(">", "Touch Play to start OpMode");
        Global.telemetry.update();
    }

    /*
     Manually set the camera gain and exposure.
     This can only be called AFTER calling initAprilTag(), and only works for Webcams;
    */
    private void    setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            Global.telemetry.addData("Camera", "Waiting");
            Global.telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            Global.telemetry.addData("Camera", "Ready");
            Global.telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested())
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }

    private boolean isDesiredTag(int tagId) {
        if (IS_BLUE_SIDE) {
            for (int desiredId : BLUE_DESIRED_TAG_IDS) {
                if (tagId == desiredId) {
                    return true;
                }
            }
        } else {
            for (int desiredId : RED_DESIRED_TAG_IDS) {
                if (tagId == desiredId) {
                    return true;
                }
            }
        }
        return false;
    }

    private Pose2d getRobotPoseFromTag(AprilTagDetection tag){
        Global.telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);

        Global.telemetry.addLine();
        Pose2d cameraRelativeOffset = getCameraRelativeOffset(tag);

        Global.telemetry.addData("Heading error","%3.0f degrees", tag.ftcPose.bearing);
        Global.telemetry.addData("Yaw error","%3.0f degrees", tag.ftcPose.yaw);
        Global.telemetry.addData("X error","%5.1f inches", cameraRelativeOffset.getX());
        Global.telemetry.addData("Y error","%5.1finches", cameraRelativeOffset.getY());

        Pose2d robotRelativeOffset = getRobotRelativeOffset(cameraRelativeOffset);
        Global.telemetry.addLine();
        Global.telemetry.addData("Robot relative offset", " x: %5.1f  , y: %5.1f , heading: %3.0f",
                robotRelativeOffset.getX(), robotRelativeOffset.getY(), Math.toDegrees(robotRelativeOffset.getHeading()));

        Pose2d robotPose = getRobotPositionFromOffset(robotRelativeOffset, tag);
        Global.telemetry.addData("Robot x position", "%5.1f", robotPose.getX());
        Global.telemetry.addData("Robot y position", "%5.1f",  robotPose.getY());
        Global.telemetry.addData("Robot heading position", "%3.0f",  Math.toDegrees(robotPose.getHeading()));
        return robotPose;
    }

    private Pose2d getCameraRelativeOffset(AprilTagDetection tag) {
        double x = tag.ftcPose.x;
        double y = tag.ftcPose.y;
        double headingDegrees = tag.ftcPose.yaw;  // might be store with negative sign, to be checked

        return new Pose2d(x, y, Math.toRadians(headingDegrees));
    }

    private Pose2d getRobotRelativeOffset(Pose2d cameraOffset) {

        double alpha = Math.toRadians(90)-CAMERA_TO_ROBOT_BEARING-cameraOffset.getHeading();

        double x = CAMERA_DISTANCE_FROM_ROBOT_CENTER*Math.cos(alpha);
        double y = CAMERA_DISTANCE_FROM_ROBOT_CENTER*Math.sin(alpha);

        // TODO: add logic to convert camera plane to camera heading for all positions
        return new Pose2d(x+cameraOffset.getX(), y+cameraOffset.getY(), Math.toRadians(90)-cameraOffset.getHeading());
    }

    private Pose2d getRobotPositionFromOffset(Pose2d robotOffset, AprilTagDetection tag){

        double tagX = centerStageTags.lookupTag(tag.id).fieldPosition.get(0)+FIELD_BACKDROP_X_OFFSET;
        double tagY = centerStageTags.lookupTag(tag.id).fieldPosition.get(1)+FIELD_BACKDROP_Y_OFFSET;
        // TODO: use field positions to get heading of tag instead of assuming 180 degrees
        double tagHeading = Math.toRadians(90);


        Pose2d tagPose = new Pose2d(tagX, tagY, tagHeading);


        return Pose2dGeometry.getPointInTransformedCoordinateSystem(tagPose, new Pose2d(0, 0, 0), robotOffset);
    }

}




