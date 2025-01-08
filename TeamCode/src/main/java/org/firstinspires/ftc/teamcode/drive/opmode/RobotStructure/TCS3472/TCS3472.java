package org.firstinspires.ftc.teamcode.drive.opmode.RobotStructure.TCS3472;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.util.TypeConversion;

@I2cDeviceType
@DeviceProperties(name = "TCS3472 Color Sensor", xmlTag = "TCS3472")
public class TCS3472 extends I2cDeviceSynchDevice<I2cDeviceSynch> {

    public final static I2cAddr ADDRESS_I2C_DEFAULT = I2cAddr.create7bit(0x29);

    public TCS3472(I2cDeviceSynch deviceClient, boolean deviceClientIsOwned) {
        super(deviceClient, deviceClientIsOwned);

        this.setOptimalReadWindow();
        this.deviceClient.setI2cAddress(ADDRESS_I2C_DEFAULT);

        super.registerArmingStateCallback(false);
        this.deviceClient.engage();
    }

    @Override
    public boolean doInitialize() {

        deviceClient.write8(0x00, 0x03);  // Enable both the RGBC and ALS sensors

        return true;
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Unknown;
    }

    @Override
    public String getDeviceName() {
        return "TCS3472 RGB Sensor";
    }

    public enum Register {

        CONFIG(0x0D),
        DEVICE_ID(0x12),
        STATUS(0x13),
        CDATAL(0x14),
        CDATAH(0x15),
        RDATAL(0x16),
        RDATAH(0x17),
        GDATAL(0x18),
        GDATAH(0x19),
        BDATAL(0x1A),
        BDATAH(0x1B);

        public final int bVal;

        Register(int bVal) {
            this.bVal = bVal;
        }
    }

    protected void setOptimalReadWindow() {

        I2cDeviceSynch.ReadWindow readWindow = new I2cDeviceSynch.ReadWindow(
                Register.CONFIG.bVal,
                13,
                I2cDeviceSynch.ReadMode.REPEAT);
        this.deviceClient.setReadWindow(readWindow);
    }

    protected void writeShort(final Register reg, short value) {
        deviceClient.write(reg.bVal, TypeConversion.shortToByteArray(value));
    }

    protected short readShort(Register reg) {
        return TypeConversion.byteArrayToShort(deviceClient.read(reg.bVal, 2));
    }


    //DEVICE COMMUNICATION
    public short getManufacturerIDRaw()
    {
        return deviceClient.read8(Register.DEVICE_ID.bVal);
    }

    protected short read16BitRegister(Register lowReg, Register highReg) {
        // Read low and high bytes and combine into a 16-bit value
        byte lowByte = deviceClient.read8(lowReg.bVal);
        byte highByte = deviceClient.read8(highReg.bVal);
        return (short) ((highByte << 8) | (lowByte & 0xFF));
    }

    public short getClearData() {
        return read16BitRegister(Register.CDATAL, Register.CDATAH);
    }

    // Get Red data
    public short getRedData() {
        return read16BitRegister(Register.RDATAL, Register.RDATAH);
    }

    // Get Green data
    public short getGreenData() {
        return read16BitRegister(Register.GDATAL, Register.GDATAH);
    }

    // Get Blue data
    public short getBlueData() {
        return read16BitRegister(Register.BDATAL, Register.BDATAH);
    }

    // Example telemetry output for testing
    @SuppressLint("DefaultLocale")
    public String getColorData() {
        return String.format("Clear: %d, Red: %d, Green: %d, Blue: %d",
                getClearData(),
                getRedData(),
                getGreenData(),
                getBlueData());
    }
}
