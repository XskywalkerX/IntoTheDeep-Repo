package org.firstinspires.ftc.teamcode.RobotStructure.testThings;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
@Disabled
public class testLine4r extends LinearOpMode {

    DcMotorEx linear;

    @Override
    public void runOpMode() throws InterruptedException {
        linear = hardwareMap.get(DcMotorEx.class, "linear");

        waitForStart();
        while(opModeIsActive()) {
            linear.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
            telemetry.addData("POS", linear.getCurrentPosition());
            telemetry.update();
        }
    }
}
