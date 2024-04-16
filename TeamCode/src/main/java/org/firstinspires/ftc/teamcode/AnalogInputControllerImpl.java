package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.AnalogInputController;
import com.qualcomm.robotcore.util.SerialNumber;

public class AnalogInputControllerImpl implements AnalogInputController {
    @Override
    public double getAnalogInputVoltage(int i) {
        return 0;
    }

    @Override
    public double getMaxAnalogInputVoltage() {
        return 0;
    }

    @Override
    public SerialNumber getSerialNumber() {
        return null;
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Other;
    }

    @Override
    public String getDeviceName() {
        return "Virtual AnalogInputController";
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
