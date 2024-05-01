package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.util.SerialNumber;

public class DigitalChannelControllerImpl implements DigitalChannelController {
    @Override
    public SerialNumber getSerialNumber() {return null;}

    @Override
    public DigitalChannel.Mode getDigitalChannelMode(int i) {
        return DigitalChannel.Mode.OUTPUT;
    }

    @Override
    public void setDigitalChannelMode(int i, DigitalChannel.Mode mode) {}

    @Override
    public void setDigitalChannelMode(int i, Mode mode) {}

    @Override
    public boolean getDigitalChannelState(int i) {
        return false;
    }

    @Override
    public void setDigitalChannelState(int i, boolean b) {}

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Other;
    }

    @Override
    public String getDeviceName() {
        return "Virtual DigitalChannelController";
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
