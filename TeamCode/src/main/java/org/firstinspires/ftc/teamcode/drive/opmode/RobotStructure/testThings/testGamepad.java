package org.firstinspires.ftc.teamcode.RobotStructure.testThings;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotStructure.GamepadControl;

@TeleOp
@Disabled
public class testGamepad extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        GamepadControl gamepadControl = new GamepadControl();

        gamepad1 = gamepadControl.getMovementDriver();

        waitForStart();
        while (opModeIsActive()) {

            while(gamepad1.right_trigger > 0) {
                gamepadControl.rumble(1000);
            }
        }
    }
}
