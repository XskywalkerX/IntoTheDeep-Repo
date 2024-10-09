package org.firstinspires.ftc.teamcode.drive.opmode.RobotStructure.MPU6050;


import static org.firstinspires.ftc.teamcode.drive.opmode.RobotStructure.MPU6050.MPU6050.ADDRESS_I2C_DEFAULT;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDeviceWithParameters;
import com.qualcomm.robotcore.util.TypeConversion;

@SuppressWarnings({"WeakerAccess", "unused"})

//@I2cDeviceType
//@DeviceProperties(name = "MPU6050 IMU", xmlTag = "MPU6050")
public class MPU6050Params extends I2cDeviceSynchDeviceWithParameters<I2cDeviceSynch, MPU6050Params.Parameters> {

    public static class Parameters implements Cloneable
    {
        I2cAddr i2cAddr = ADDRESS_I2C_DEFAULT;

        // All settings available
        public ACCEL_CONFIG accelConfig = ACCEL_CONFIG.AFS_SEL_4G;

        public Parameters clone()
        {
            try
            {
                return (Parameters) super.clone();
            }
            catch(CloneNotSupportedException e)
            {
                throw new RuntimeException("Internal Error: Parameters not cloneable");
            }
        }
    }

    public MPU6050Params(I2cDeviceSynch deviceClient, boolean deviceClientIsOwned) {
        super(deviceClient, deviceClientIsOwned, new Parameters());

        this.setOptimalReadWindow();
        this.deviceClient.setI2cAddress(ADDRESS_I2C_DEFAULT);

        super.registerArmingStateCallback(false);
        this.deviceClient.engage();
    }

    @Override
    public boolean internalInitialize(@NonNull Parameters params) {

        this.parameters = params.clone();

        deviceClient.setI2cAddress(params.i2cAddr);

        int configSettings = params.accelConfig.bVal;

        writeShort(Register.CONFIG, (short) configSettings);

        // Mask out alert signal bit, which we can't control
        return (readShort(Register.CONFIG) & 0xFFEF) == configSettings;
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

        FIRST(0),
        SELF_TEST_X(0x0D),
        SELF_TEST_Y(0x0E),
        SELF_TEST_Z(0x0F),
        SELF_TEST_A(0x10),
        SMPLRT_DIV(0x19),
        CONFIG(0x1A),
        GYRO_CONFIG(0x1B),
        ACCEL_CONFIG(0x1C),
        FIFO_EN(0x23),
        I2C_MST_CTRL(0x24),
        I2C_SLV0_ADDR(0x25),
        I2C_SLV0_REG(0x26),
        I2C_SLV0_CTRL(0x27),
        I2C_SLV1_ADDR(0x28),
        I2C_SLV1_REG(0x29),
        I2C_SLV1_CTRL(0x2A),
        I2C_SLV2_ADDR(0x2B),
        I2C_SLV2_REG(0x2C),
        I2C_SLV2_CTRL(0x2D),
        I2C_SLV3_ADDR(0x2E),
        I2C_SLV3_REG(0x2F),
        I2C_SLV3_CTRL(0x30),
        I2C_SLV4_ADDR(0x31),
        I2C_SLV4_REG(0x32),
        I2C_SLV4_DO(0x33),
        I2C_SLV4_CTRL(0x34),
        I2C_SLV4_DI(0x35),
        I2C_MST_STATUS(0x36),
        INT_PIN_CFG(0x37),
        INT_ENABLE(0x38),
        INT_STATUS(0x3A),
        ACCEL_XOUT_H(0x3B),
        ACCEL_XOUT_L(0x3C),
        ACCEL_YOUT_H(0x3D),
        ACCEL_YOUT_L(0x3E),
        ACCEL_ZOUT_H(0x3F),
        ACCEL_ZOUT_L(0x40),
        TEMP_OUT_H(0x41),
        TEMP_OUT_L(0x42),
        GYRO_XOUT_H(0x43),
        GYRO_XOUT_L(0x44),
        GYRO_YOUT_H(0x45),
        GYRO_YOUT_L(0x46),
        GYRO_ZOUT_H(0x47),
        GYRO_ZOUT_L(0x48),
        EXT_SENS_DATA_00(0x49),
        EXT_SENS_DATA_01(0x4A),
        EXT_SENS_DATA_02(0x4B),
        EXT_SENS_DATA_03(0x4C),
        EXT_SENS_DATA_04(0x4D),
        EXT_SENS_DATA_05(0x4E),
        EXT_SENS_DATA_06(0x4F),
        EXT_SENS_DATA_07(0x50),
        EXT_SENS_DATA_08(0x51),
        EXT_SENS_DATA_09(0x52),
        EXT_SENS_DATA_10(0x53),
        EXT_SENS_DATA_11(0x54),
        EXT_SENS_DATA_12(0x55),
        EXT_SENS_DATA_13(0x56),
        EXT_SENS_DATA_14(0x57),
        EXT_SENS_DATA_15(0x58),
        EXT_SENS_DATA_16(0x59),
        EXT_SENS_DATA_17(0x5A),
        EXT_SENS_DATA_18(0x5B),
        EXT_SENS_DATA_19(0x5C),
        EXT_SENS_DATA_20(0x5D),
        EXT_SENS_DATA_21(0x5E),
        EXT_SENS_DATA_22(0x5F),
        EXT_SENS_DATA_23(0x60),
        I2C_SLV0_DO(0x63),
        I2C_SLV1_DO(0x64),
        I2C_SLV2_DO(0x65),
        I2C_SLV3_DO(0x66),
        I2C_MST_DELAY_CTRL(0x67),
        SIGNAL_PATH_RESET(0x68),
        USER_CTRL(0x6A),
        PWR_MGMT_1(0x6B),
        PWR_MGMT_2(0x6C),
        FIFO_COUNTH(0x72),
        FIFO_COUNTL(0x73),
        FIFO_R_W(0x74),
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
                Register.FIRST.bVal,
                Register.LAST.bVal,
                I2cDeviceSynch.ReadMode.REPEAT);
        this.deviceClient.setReadWindow(readWindow);
    }

    protected void writeShort(final Register reg, short value) {
        deviceClient.write(reg.bVal, TypeConversion.shortToByteArray(value));
    }

    protected short readShort(Register reg) {
        return TypeConversion.byteArrayToShort(deviceClient.read(reg.bVal, 2));
    }
}
