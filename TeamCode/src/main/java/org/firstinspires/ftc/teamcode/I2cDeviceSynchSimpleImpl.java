package org.firstinspires.ftc.teamcode;

import androidx.annotation.Nullable;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;
import com.qualcomm.robotcore.hardware.I2cWaitControl;
import com.qualcomm.robotcore.hardware.TimestampedData;

public class I2cDeviceSynchSimpleImpl implements I2cDeviceSynchSimple {
    @Override
    public byte read8() {
        return 0;
    }

    @Override
    public byte read8(int i) {
        return 0;
    }

    @Override
    public byte[] read(int i) {
        return new byte[0];
    }

    @Override
    public byte[] read(int i, int i1) {
        return new byte[0];
    }

    @Override
    public TimestampedData readTimeStamped(int i) {
        return new TimestampedData();
    }

    @Override
    public TimestampedData readTimeStamped(int i, int i1) {
        return new TimestampedData();
    }

    @Override
    public void write8(int i) {

    }

    @Override
    public void write8(int i, int i1) {

    }

    @Override
    public void write(byte[] bytes) {

    }

    @Override
    public void write(int i, byte[] bytes) {

    }

    @Override
    public void write8(int i, I2cWaitControl i2cWaitControl) {

    }

    @Override
    public void write8(int i, int i1, I2cWaitControl i2cWaitControl) {

    }

    @Override
    public void write(byte[] bytes, I2cWaitControl i2cWaitControl) {

    }

    @Override
    public void write(int i, byte[] bytes, I2cWaitControl i2cWaitControl) {

    }

    @Override
    public void waitForWriteCompletions(I2cWaitControl i2cWaitControl) {

    }

    @Override
    public void enableWriteCoalescing(boolean b) {

    }

    @Override
    public boolean isWriteCoalescingEnabled() {
        return false;
    }

    @Override
    public boolean isArmed() {
        return false;
    }

    @Override
    public void setI2cAddr(I2cAddr i2cAddr) {

    }

    @Override
    public I2cAddr getI2cAddr() {
        return null;
    }

    @Override
    public void setLogging(boolean b) {

    }

    @Override
    public boolean getLogging() {
        return false;
    }

    @Override
    public void setLoggingTag(String s) {

    }

    @Override
    public String getLoggingTag() {
        return "Virtual I2C";
    }

    @Override
    public void setUserConfiguredName(@Nullable String s) {

    }

    @Nullable
    @Override
    public String getUserConfiguredName() {
        return null;
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Unknown;
    }

    @Override
    public String getDeviceName() {
        return "Virtual I2cDeviceSynchSimple";
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
    public void setHealthStatus(HealthStatus healthStatus) {

    }

    @Override
    public HealthStatus getHealthStatus() {
        return HealthStatus.UNKNOWN;
    }

    @Override
    public void setI2cAddress(I2cAddr i2cAddr) {

    }

    @Override
    public I2cAddr getI2cAddress() {
        return new I2cAddr(0);
    }
}
