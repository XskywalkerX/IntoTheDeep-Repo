package org.firstinspires.ftc.teamcode.drive.opmode.RobotStructure.Gustavo;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.opmode.RR.SampleMecanumDrive;
@TeleOp
public class Gustavolinear extends LinearOpMode {

    SampleMecanumDrive robot;
    DcMotorEx motor_linearcima;

    Servo servoitk;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new SampleMecanumDrive(hardwareMap);
        motor_linearcima = hardwareMap.get(DcMotorEx.class, "motor_linear_cima");
        servoitk = hardwareMap.get(Servo.class, "servoitk");

        waitForStart();

        while(opModeIsActive()){

            motor_linearcima.setPower(gamepad1.right_trigger - gamepad1.left_trigger);



            if(gamepad1.a){
                servoitk.setPosition(1);
            } else if (gamepad1.b){
                servoitk.setPosition(0);
            }
            robot.setWeightedDrivePower(new Pose2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    (gamepad1.right_stick_y - gamepad1.right_stick_x)
            ));
        }

    }
}
