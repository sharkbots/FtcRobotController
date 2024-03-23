package org.firstinspires.ftc.teamcode.tools;


import android.media.Image;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PlaneLauncher {
    private final DcMotor planeLauncher;
    private final Servo planeAngle;
    private final Button handlerX;
    private boolean isPlaneLaunchTriggered, isPlaneTimerReset;
    private final double planeAngleStore, planeAngleLaunch;

    private final ElapsedTime timerForPlane;

    public PlaneLauncher(HardwareMap hardwareMap, Button handlerX){
        // Launcher
        planeLauncher = hardwareMap.dcMotor.get("planeLauncher");
        planeLauncher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Angle
        planeAngle = hardwareMap.servo.get("planeAngle");
        planeAngle.scaleRange(0.56, 0.77);
        planeAngleStore = 1;
        planeAngleLaunch = 0;

        // Timer for launching
        timerForPlane = new ElapsedTime();
        isPlaneTimerReset = false;
        isPlaneLaunchTriggered = false;

        this.handlerX = handlerX;
    }

    public PlaneLauncher(HardwareMap hardwareMap){
        // Launcher
        planeLauncher = hardwareMap.dcMotor.get("planeLauncher");
        planeLauncher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Angle
        planeAngle = hardwareMap.servo.get("planeAngle");
        planeAngle.scaleRange(0.56, 0.77);
        planeAngleStore = 1;
        planeAngleLaunch = 0;

        timerForPlane = null;
        handlerX = null;
    }

    public void load(){
        planeAngle.setPosition(planeAngleStore);
    }
    public void update(){
        if(handlerX.On()) {
            isPlaneLaunchTriggered = true;
            if(!isPlaneTimerReset) {
                timerForPlane.reset();
                isPlaneTimerReset = true;
            }

            planeAngle.setPosition(planeAngleLaunch);

            if(timerForPlane.milliseconds() >= 500) {
                planeLauncher.setPower(1);
                timerForPlane.reset();
                isPlaneTimerReset = true;
            }
        }
        else{
            if(isPlaneLaunchTriggered &&  timerForPlane.milliseconds() >= 500) {
                planeLauncher.setPower(1);
                isPlaneLaunchTriggered = false;
                timerForPlane.reset();
                isPlaneTimerReset = true;
            }
            else {
                if(timerForPlane.milliseconds() >= 500) {
                    isPlaneTimerReset = false;
                    planeAngle.setPosition(planeAngleStore);
                    planeLauncher.setPower(0);
                    isPlaneLaunchTriggered = false;
                }
            }
        }
    }
}
