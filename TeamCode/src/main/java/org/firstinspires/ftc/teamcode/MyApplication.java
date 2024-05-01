package org.firstinspires.ftc.teamcode;

import android.app.Application;
import android.util.Log;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.ServoConfigurationType;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.tools.Robot;


public class MyApplication extends Application {
    @Override
    public void onCreate() {
        super.onCreate();
        Log.d("PLAYGROUND", "Init SharkBots");
        AppUtil.onApplicationStart(this); // Necessary to make FTC Libraries work

        HardwareMap vhdmap = new HardwareMap(getApplicationContext(), null);
        Log.d("PLAYGROUND", "Init VirtualDevices");

        createVirtualDevices(vhdmap);
        Log.d("PLAYGROUND", "Init Robot");

        Robot bot = new Robot(vhdmap, new Gamepad(), new Gamepad());
        Log.d("PLAYGROUND", "Init Finished");


    }
    void createVirtualDevices(HardwareMap vhdmap) {
        // Lift
        vhdmap.dcMotor.put("liftMotor", new DcMotorImplEx(new DcMotorControllerExImpl(), 1, DcMotorSimple.Direction.FORWARD, new MotorConfigurationType()));
        vhdmap.touchSensor.put("liftTouchDown",  new TouchSensorImpl());

        // Claw
        vhdmap.servo.put("clawPitch", new ServoImplEx(new ServoControllerExImpl(), 1, ServoImplEx.Direction.FORWARD, new ServoConfigurationType()));
        vhdmap.servo.put("clawYaw", new ServoImplEx(new ServoControllerExImpl(), 1, ServoImplEx.Direction.FORWARD, new ServoConfigurationType()));
        vhdmap.servo.put("clawGrip", new ServoImplEx(new ServoControllerExImpl(), 1, ServoImplEx.Direction.FORWARD, new ServoConfigurationType()));
        vhdmap.put("rotationPositionInput", new AnalogInput(new AnalogInputControllerImpl(), 1));
        vhdmap.put("armPositionInput", new AnalogInput(new AnalogInputControllerImpl(), 2));

        //PlaneLauncher
        vhdmap.servo.put("planeLauncher", new ServoImplEx(new ServoControllerExImpl(), 1, ServoImplEx.Direction.FORWARD, new ServoConfigurationType()));

        //Hanger
        vhdmap.dcMotor.put("skyHookMotor", new DcMotorImplEx(new DcMotorControllerExImpl(), 1, DcMotorSimple.Direction.FORWARD, new MotorConfigurationType()));
        vhdmap.dcMotor.put("frontRightMotor", new DcMotorImplEx(new DcMotorControllerExImpl(), 1, DcMotorSimple.Direction.FORWARD, new MotorConfigurationType()));

        //Intake
        vhdmap.dcMotor.put("intakeMotor", new DcMotorImplEx(new DcMotorControllerExImpl(), 1, DcMotorSimple.Direction.FORWARD, new MotorConfigurationType()));
        vhdmap.servo.put("intakeFlipper", new ServoImplEx(new ServoControllerExImpl(), 1, ServoImplEx.Direction.FORWARD, new ServoConfigurationType()));
        for(int i=0; i<4;i++) {
            vhdmap.put("led" + (i + 1) + "red", new LED(new DigitalChannelControllerImpl(), i));
            vhdmap.put("led" + (i + 1) + "green", new LED(new DigitalChannelControllerImpl(), i));
        }
        vhdmap.put("colorsensor1", new RevColorSensorV3(new I2cDeviceSynchSimpleImpl(), true));
        vhdmap.put("colorsensor2", new RevColorSensorV3(new I2cDeviceSynchSimpleImpl(), true));

        //LEDRibbons
        vhdmap.put("blinkin1", new RevBlinkinLedDriver(new ServoControllerExImpl(), 1));
        vhdmap.put("blinkin2", new RevBlinkinLedDriver(new ServoControllerExImpl(), 1));
        vhdmap.dcMotor.put("blinkinpower", new DcMotorImplEx(new DcMotorControllerExImpl(), 1, DcMotorSimple.Direction.FORWARD, new MotorConfigurationType()));
    }
}