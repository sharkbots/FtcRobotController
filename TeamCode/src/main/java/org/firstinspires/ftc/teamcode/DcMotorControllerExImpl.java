package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class DcMotorControllerExImpl implements DcMotorControllerEx {
    @Override
    public void setMotorType(int i, MotorConfigurationType motorConfigurationType) {

    }

    @Override
    public MotorConfigurationType getMotorType(int i) {
        return new MotorConfigurationType();
    }

    @Override
    public void setMotorMode(int i, DcMotor.RunMode runMode) {

    }

    @Override
    public DcMotor.RunMode getMotorMode(int i) {
        return DcMotor.RunMode.RUN_WITHOUT_ENCODER;
    }

    @Override
    public void setMotorPower(int i, double v) {

    }

    @Override
    public double getMotorPower(int i) {
        return 0;
    }

    @Override
    public boolean isBusy(int i) {
        return false;
    }

    @Override
    public void setMotorZeroPowerBehavior(int i, DcMotor.ZeroPowerBehavior zeroPowerBehavior) {

    }

    @Override
    public DcMotor.ZeroPowerBehavior getMotorZeroPowerBehavior(int i) {
        return DcMotor.ZeroPowerBehavior.FLOAT;
    }

    @Override
    public boolean getMotorPowerFloat(int i) {
        return false;
    }

    @Override
    public void setMotorTargetPosition(int i, int i1) {

    }

    @Override
    public int getMotorTargetPosition(int i) {
        return 0;
    }

    @Override
    public int getMotorCurrentPosition(int i) {
        return 0;
    }

    @Override
    public void resetDeviceConfigurationForOpMode(int i) {

    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Other;
    }

    @Override
    public String getDeviceName() {
        return "Virtual DC Motor Controller";
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

    @Override
    public void setMotorEnable(int i) {

    }

    @Override
    public void setMotorDisable(int i) {

    }

    @Override
    public boolean isMotorEnabled(int i) {
        return false;
    }

    @Override
    public void setMotorVelocity(int i, double v) {

    }

    @Override
    public void setMotorVelocity(int i, double v, AngleUnit angleUnit) {

    }

    @Override
    public double getMotorVelocity(int i) {
        return 0;
    }

    @Override
    public double getMotorVelocity(int i, AngleUnit angleUnit) {
        return 0;
    }

    @Override
    public void setPIDCoefficients(int i, DcMotor.RunMode runMode, PIDCoefficients pidCoefficients) {

    }

    @Override
    public void setPIDFCoefficients(int i, DcMotor.RunMode runMode, PIDFCoefficients pidfCoefficients) throws UnsupportedOperationException {

    }

    @Override
    public PIDCoefficients getPIDCoefficients(int i, DcMotor.RunMode runMode) {
        return new PIDCoefficients();
    }

    @Override
    public PIDFCoefficients getPIDFCoefficients(int i, DcMotor.RunMode runMode) {
        return new PIDFCoefficients();
    }

    @Override
    public void setMotorTargetPosition(int i, int i1, int i2) {

    }

    @Override
    public double getMotorCurrent(int i, CurrentUnit currentUnit) {
        return 0;
    }

    @Override
    public double getMotorCurrentAlert(int i, CurrentUnit currentUnit) {
        return 0;
    }

    @Override
    public void setMotorCurrentAlert(int i, double v, CurrentUnit currentUnit) {

    }

    @Override
    public boolean isMotorOverCurrent(int i) {
        return false;
    }
}
