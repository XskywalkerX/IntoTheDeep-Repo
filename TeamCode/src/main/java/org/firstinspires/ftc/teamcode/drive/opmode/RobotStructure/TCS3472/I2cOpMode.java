package org.firstinspires.ftc.teamcode.drive.opmode.RobotStructure.TCS3472;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class I2cOpMode extends LinearOpMode {

    TCS3472 colorSensor;

    @Override
    public void runOpMode() throws InterruptedException {

        colorSensor = hardwareMap.get(TCS3472.class, "i2cColor");

        waitForStart();
        while (opModeIsActive()) {

            double r = colorSensor.getRedData();
            double g = colorSensor.getGreenData();
            double b = colorSensor.getBlueData();

            telemetry.addData("Device ID", colorSensor.getManufacturerIDRaw());
            telemetry.addData("RED", r);
            telemetry.addData("GREEN", g);
            telemetry.addData("BLUE", b);
            telemetry.update();
        }
    }
}
