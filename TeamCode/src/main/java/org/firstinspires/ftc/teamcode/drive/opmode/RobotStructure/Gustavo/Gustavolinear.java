package org.firstinspires.ftc.teamcode.RobotStructure.Gustavo;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.RR.SampleMecanumDrive;

public class Gustavolinear extends LinearOpMode {

    SampleMecanumDrive robot;
    DcMotorEx motor_linearfr;
    DcMotorEx motor_linearcima;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new SampleMecanumDrive(hardwareMap);
        motor_linearcima = hardwareMap.get(DcMotorEx.class, "motorlc");


    }
}
