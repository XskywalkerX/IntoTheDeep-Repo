package org.firstinspires.ftc.teamcode.RobotStructure.TCS3472;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;

import java.util.Map;

@TeleOp
@Disabled
public class I2cOpMode extends LinearOpMode {

    TCS3472 colorSensor;
    TCS3472 i2c;

    @Override
    public void runOpMode() throws InterruptedException {

        i2c = hardwareMap.get(TCS3472.class, "i2cColor");
        colorSensor = new TCS3472(i2c.getDeviceClient(), false);

        while(!isStarted()) {
            colorSensor.initialize();
        }

        waitForStart();
        while (opModeIsActive()) {

            colorSensor.setOptimalReadWindow();

            double r = colorSensor.getRedData();
            double g = colorSensor.getGreenData();
            double b = colorSensor.getBlueData();

            telemetry.addData("ENABLE", colorSensor.getEnable());
            telemetry.addData("Device ID", colorSensor.getManufacturerIDRaw());
            telemetry.addData("RED", r);
            telemetry.addData("GREEN", g);
            telemetry.addData("BLUE", b);
            telemetry.addData("CLEAR", colorSensor.getClearData());
            telemetry.update();
        }
    }
}
