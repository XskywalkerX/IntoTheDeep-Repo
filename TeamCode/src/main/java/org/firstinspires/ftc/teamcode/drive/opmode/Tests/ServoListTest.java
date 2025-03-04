package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;


@Autonomous
public class ServoListTest extends LinearOpMode {

    Servo se1, se2, se3, se4, se5, se6;
    Servo sc1, sc2, sc3, sc4, sc5, sc6;

    @Override
    public void runOpMode() throws InterruptedException {
        se1 = hardwareMap.get(Servo.class, "se1");
        se2 = hardwareMap.get(Servo.class, "se2");
        se3 = hardwareMap.get(Servo.class, "se3");
        se4 = hardwareMap.get(Servo.class, "se4");
        se5 = hardwareMap.get(Servo.class, "se5");
        se6 = hardwareMap.get(Servo.class, "se6");
        sc1 = hardwareMap.get(Servo.class, "sc1");
        sc2 = hardwareMap.get(Servo.class, "sc2");
        sc3 = hardwareMap.get(Servo.class, "sc3");
        sc4 = hardwareMap.get(Servo.class, "sc4");
        sc5 = hardwareMap.get(Servo.class, "sc5");
        sc6 = hardwareMap.get(Servo.class, "sc6");

        Servo[] servos = {se1, se2, se3, se4, se5, se6, sc1, sc2, sc3, sc4, sc5, sc6};

        ElapsedTime timer = new ElapsedTime();

        waitForStart();

        while (opModeIsActive()) {
            if (timer.seconds() >= 0.5) {
                for (Servo servo : servos) {
                    if (servo.getPosition() == 1) {
                        servo.setPosition(0);
                    } else {
                        servo.setPosition(1);
                    }
                    telemetry.addData("SErvo position ", servo.getPosition());
                }
                timer.reset();
            }
            telemetry.addData("Timer seconds ", timer.seconds());
            telemetry.update();
        }
    }
}
