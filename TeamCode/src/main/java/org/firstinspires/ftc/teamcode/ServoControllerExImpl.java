package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.ServoConfigurationType;

public class ServoControllerExImpl implements ServoControllerEx {
    @Override
    public void setServoPwmRange(int i, @NonNull PwmControl.PwmRange pwmRange) {

    }

    @NonNull
    @Override
    public PwmControl.PwmRange getServoPwmRange(int i) {
        return new PwmControl.PwmRange(0.0, 1.0);
    }

    @Override
    public void setServoPwmEnable(int i) {

    }

    @Override
    public void setServoPwmDisable(int i) {

    }

    @Override
    public boolean isServoPwmEnabled(int i) {
        return false;
    }

    @Override
    public void setServoType(int i, ServoConfigurationType servoConfigurationType) {

    }

    @Override
    public void pwmEnable() {

    }

    @Override
    public void pwmDisable() {

    }

    @Override
    public PwmStatus getPwmStatus() {
        return PwmStatus.DISABLED;
    }

    @Override
    public void setServoPosition(int i, double v) {

    }

    @Override
    public double getServoPosition(int i) {
        return 0;
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Other;
    }

    @Override
    public String getDeviceName() {
        return "Virtual ServoControllerEx";
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
