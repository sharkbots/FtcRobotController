package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.tools.Button;
import org.firstinspires.ftc.teamcode.tools.ServoActionManager;

public class PlaneLauncher {
    private final Servo planeLauncher;
    private final Button handlerX;
    private final double holdPlane, releasePlane;
    private final ServoActionManager planeLauncherServoActionManager;


    public PlaneLauncher(HardwareMap hardwareMap, Button handlerX) {
        // Launcher
        planeLauncher = hardwareMap.servo.get("planeLauncher");
        holdPlane = 0.77;
        releasePlane = 0.2;

        planeLauncherServoActionManager = new ServoActionManager(planeLauncher);

        this.handlerX = handlerX;
    }


    public void releasePlane(){
        planeLauncherServoActionManager.setServoPosition(releasePlane);
    }
    public void storePlane(){
        planeLauncherServoActionManager.setServoPosition(holdPlane);
    }

    public void update() {
        if (handlerX.Pressed()){
            releasePlane();
        }
    }
}