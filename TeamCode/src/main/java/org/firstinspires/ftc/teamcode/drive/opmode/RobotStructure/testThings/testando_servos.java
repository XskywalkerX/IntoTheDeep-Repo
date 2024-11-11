package org.firstinspires.ftc.teamcode.drive.opmode.RobotStructure.testThings;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.opmode.RR.SampleMecanumDrive;

@TeleOp
public class testando_servos extends LinearOpMode {
    //Servo servo_left;

    boolean mlc = false;

    DcMotorEx motor_linear_cima;
    SampleMecanumDrive drive;
    //Servo servo_itk;
    public static double servoL_position = 0;
    public static double MULTIPLIER = 0.05;
    @Override
    public void runOpMode() throws InterruptedException {
        //servo_left = hardwareMap.get(Servo.class, "servo_left");
       // servo_itk = hardwareMap.get(Servo.class, "itk");
        //servo_left.setDirection(Servo.Direction.REVERSE);
        drive = new SampleMecanumDrive(hardwareMap);
        motor_linear_cima = hardwareMap.get(DcMotorEx.class, "motor_linear_cima");



        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        waitForStart();

        while(opModeIsActive()){


            if(gamepad1.y){
                mlc = true;
            } else {
                mlc = false;
            }

            if (mlc){
                motor_linear_cima.setPower(gamepad1.right_trigger-gamepad1.left_trigger);
            } else{
               // servo_itk.setPosition(gamepad1.right_trigger - gamepad1.left_trigger);
            }


            //servoL_position = linear.getCurrentPosition() * MULTIPLIER;

            //servo_left.setPosition(servoL_position);
            if (gamepad1.a){
                servoL_position = 1;
            } else if (gamepad1.b){
                servoL_position=0;
            }

            drive.setWeightedDrivePower(
                    new Pose2d(
                            gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );


            telemetry.addData("servoL_position", servoL_position);
            telemetry.addData("MLC", mlc);
            telemetry.update();
        }
    }
}
