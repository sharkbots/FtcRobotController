package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.TouchSensor;

public class TouchSensorImpl implements TouchSensor {
    @Override
    public double getValue() {
        return 0;
    }

    @Override
    public boolean isPressed() {
        return false;
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Other;
    }

    @Override
    public String getDeviceName() {
        return "Virtual Touch Sensor";
    }

    @Override
    public String getConnectionInfo() {
        return "Virtual Connection";
    }

    @Override
    public int getVersion() {
        return 0;
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {

    }

    @Override
    public void close() {

    }
}
