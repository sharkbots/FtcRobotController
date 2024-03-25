package org.firstinspires.ftc.teamcode.tools;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;
public class ServoActionManager {
    private final Servo servo;
    private final AnalogInput analogSensor;

    public ServoActionManager(Servo servo){
        this.servo = servo;
        this.analogSensor = null;
    }

    public ServoActionManager(Servo servo, AnalogInput analogSensor){
        this.servo = servo;
        this.analogSensor = analogSensor;
    }


    public void setServoPosition(double position){
        servo.setPosition(position);
    }

    public boolean waitForAnalogServoSensorAtPosition(double position, double scale, double tolerance){
        assert analogSensor != null;
        double currentPosition = analogSensor.getVoltage() / scale * 360;
        return Math.abs(position - currentPosition) <= tolerance;
    }
}
