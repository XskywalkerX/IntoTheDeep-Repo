package org.firstinspires.ftc.teamcode.drive.opmode.RobotStructure.MPU6050;


import static java.lang.Math.PI;
import static java.lang.Math.atan2;
import static java.lang.Math.sqrt;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.util.TypeConversion;

@SuppressWarnings({"WeakerAccess", "unused"})

@I2cDeviceType
@DeviceProperties(name = "MPU6050 IMU", xmlTag = "MPU6050")
public class MPU6050 extends I2cDeviceSynchDevice<I2cDeviceSynch> {

    public final static I2cAddr ADDRESS_I2C_DEFAULT = I2cAddr.create7bit(0x68);

    public MPU6050(I2cDeviceSynch deviceClient, boolean deviceClientIsOwned) {
        super(deviceClient, deviceClientIsOwned);

        this.setOptimalReadWindow();
        this.deviceClient.setI2cAddress(ADDRESS_I2C_DEFAULT);

        super.registerArmingStateCallback(false);
        this.deviceClient.engage();
    }

    @Override
    public boolean doInitialize() {

        int accelSettings = ACCEL_CONFIG.AFS_SEL_2G.bVal;

        writeShort(Register.CONFIG, (short) accelSettings);

        return (readShort(Register.CONFIG) & 0xFFEF) == accelSettings;
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Unknown;
    }

    @Override
    public String getDeviceName() {
        return "MPU6050 GY-521 IMU";
    }

    public enum Register {

        CONFIG(0x1A),
        GYRO_CONFIG(0x1B),
        ACCEL_CONFIG(0x1C),
        ACCEL_XOUT_H(0x3B),
        ACCEL_XOUT_L(0x3C),
        ACCEL_YOUT_H(0x3D),
        ACCEL_YOUT_L(0x3E),
        ACCEL_ZOUT_H(0x3F),
        FIFO_EN(0x23),
        ACCEL_ZOUT_L(0x40),
        GYRO_XOUT_H(0x43),
        GYRO_XOUT_L(0x44),
        GYRO_YOUT_H(0x45),
        GYRO_YOUT_L(0x46),
        GYRO_ZOUT_H(0x47),
        GYRO_ZOUT_L(0x48),
        WHO_AM_I(0x75),
        LAST(WHO_AM_I.bVal);

        public final int bVal;

        Register(int bVal) {
            this.bVal = bVal;
        }
    }
    public enum ACCEL_CONFIG {

        AFS_SEL_2G(0x0000),   // ±2g range
        AFS_SEL_4G(0x0008),   // ±4g range
        AFS_SEL_8G(0x0010),   // ±8g range
        AFS_SEL_16G(0x0018);  // ±16g range

        public final int bVal;

        ACCEL_CONFIG(int bVal) {
            this.bVal = bVal;
        }

    }

    protected void setOptimalReadWindow() {

        I2cDeviceSynch.ReadWindow readWindow = new I2cDeviceSynch.ReadWindow(
                Register.CONFIG.bVal,
                26,
                I2cDeviceSynch.ReadMode.REPEAT);
        this.deviceClient.setReadWindow(readWindow);
    }

    protected void writeShort(final Register reg, short value) {
        deviceClient.write(reg.bVal, TypeConversion.shortToByteArray(value));
    }

    protected short readShort(Register reg) {
        return TypeConversion.byteArrayToShort(deviceClient.read(reg.bVal, 2));
    }

    public short getManufacturerIDRaw() {
        return deviceClient.read8(Register.WHO_AM_I.bVal);
    }

    public short getXAccel() {
        return readShort(Register.ACCEL_XOUT_L);
    }

    public byte getXACCELB() {
        return deviceClient.read8(Register.ACCEL_XOUT_L.bVal);
    }

    public short getYAccel() {
        return readShort(Register.ACCEL_YOUT_L);
    }

    public short getZAccel() {
        return readShort(Register.ACCEL_ZOUT_L);
    }

    private double convertRawAccel(short rawValue) {
        final double ACCEL_SCALE = 16384.0;
        return rawValue / ACCEL_SCALE;
    }

    public double Pitch() {
        short xRaw = getXAccel();
        short yRaw = getYAccel();
        short zRaw = getZAccel();

        double xAccel = convertRawAccel(xRaw);
        double yAccel = convertRawAccel(yRaw);
        double zAccel = convertRawAccel(zRaw);

        return atan2(yAccel, sqrt((xAccel * xAccel) + (zAccel * zAccel))) * 180 / PI;
    }

    public double Roll() {
        short xRaw = getXAccel();
        short yRaw = getYAccel();
        short zRaw = getZAccel();

        double xAccel = convertRawAccel(xRaw);
        double yAccel = convertRawAccel(yRaw);
        double zAccel = convertRawAccel(zRaw);

        return atan2(xAccel, sqrt((yAccel * yAccel) + (zAccel * zAccel))) * 180 / PI;
    }
}
