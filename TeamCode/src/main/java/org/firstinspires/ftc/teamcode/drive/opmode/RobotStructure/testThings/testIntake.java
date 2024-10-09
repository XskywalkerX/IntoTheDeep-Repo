package org.firstinspires.ftc.teamcode.drive.opmode.RobotStructure.testThings;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class testIntake extends LinearOpMode {

    DistanceSensor sensor;
    DcMotorEx itk;

    public static boolean EVERYWHERE = false;
    public static double DISTANCE = 5;

    @Override
    public void runOpMode() throws InterruptedException {

        sensor = hardwareMap.get(DistanceSensor.class, "sensor");
        itk = hardwareMap.get(DcMotorEx.class, "itk");

        itk.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        while(opModeIsActive()) {
            EVERYWHERE = sensor.getDistance(DistanceUnit.CM) < DISTANCE;
            itk.setPower(EVERYWHERE ? 0 : 1);
        }
    }
}
