package org.firstinspires.ftc.teamcode.drive.opmode.PedroPathing.Robot.OpModes;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.follower.Follower;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.opmode.PedroPathing.Robot.Systems.DeliverySystem;
import org.firstinspires.ftc.teamcode.drive.opmode.PedroPathing.Robot.Systems.IntakeSystem;
import org.firstinspires.ftc.teamcode.drive.opmode.PedroPathing.Robot.Systems.LinearLevel;
import org.firstinspires.ftc.teamcode.drive.opmode.PedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.drive.opmode.PedroPathing.constants.LConstants;

@TeleOp
public class TeleOpLoop extends LinearOpMode {

    ElapsedTime time = new ElapsedTime();

    LinearLevel linearLevel = LinearLevel.GROUND;

    DeliverySystem deliverySystem;
    IntakeSystem intakeSystem;
    Follower follower;

    GamepadEx gamepad_1, gamepad_2;

    @Override
    public void runOpMode() throws InterruptedException {
        Constants.setConstants(FConstants.class, LConstants.class);

        FConstants.readData();
        LConstants.readData();

        deliverySystem = new DeliverySystem(hardwareMap, telemetry);
        intakeSystem = new IntakeSystem(hardwareMap, telemetry);
        follower = new Follower(hardwareMap);

        gamepad_1 = new GamepadEx(gamepad1);
        gamepad_2 = new GamepadEx(gamepad2);

        waitForStart();
        while(opModeIsActive()) {
            intakeSystem.getExpansion().setPower(
                    gamepad_2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) - gamepad_2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER));

            intakeSystem.controlIntake();

            deliverySystem.getLinear().setTargetPosition(linearLevel.getPosition());
            deliverySystem.moveLinear(time);

            switch (linearLevel) {
                case GROUND:

                    deliverySystem.moveBox(1.0);

                    if(!deliverySystem.getLinear().isBusy()) {
                        if(gamepad_2.wasJustPressed(GamepadKeys.Button.A)) {
                            linearLevel = LinearLevel.LOW_BASKET;
                        } else if (gamepad_2.wasJustPressed(GamepadKeys.Button.B)) {
                            linearLevel = LinearLevel.CLIP;
                        } else if (gamepad_2.wasJustPressed(GamepadKeys.Button.Y)) {
                            linearLevel = LinearLevel.HIGH_BASKET;
                        }
                    } else {
                        if(deliverySystem.getLinear().getCurrentPosition() < 1000) {
                            deliverySystem.openClaw();
                        }
                    }
                    break;
                case LOW_BASKET:

                    deliverySystem.moveBox((double) deliverySystem.getLinear().getCurrentPosition() / linearLevel.getPosition());

                    if (!deliverySystem.getLinear().isBusy()) {
                        if(gamepad_2.wasJustPressed(GamepadKeys.Button.A)) {
                            linearLevel = LinearLevel.GROUND;
                        } else if (gamepad_2.wasJustPressed(GamepadKeys.Button.B)) {
                            linearLevel = LinearLevel.CLIP;
                        } else if (gamepad_2.wasJustPressed(GamepadKeys.Button.Y)) {
                            linearLevel = LinearLevel.HIGH_BASKET;
                        }
                    }
                    break;
                case CLIP:

                    deliverySystem.moveBox(1.0);

                    if (!deliverySystem.getLinear().isBusy()) {
                        if(gamepad_2.wasJustPressed(GamepadKeys.Button.A)) {
                            linearLevel = LinearLevel.GROUND;
                        } else if (gamepad_2.wasJustPressed(GamepadKeys.Button.B)) {
                            linearLevel = LinearLevel.LOW_BASKET;
                        } else if (gamepad_2.wasJustPressed(GamepadKeys.Button.Y)) {
                            linearLevel = LinearLevel.HIGH_BASKET;
                        }
                    }

                    if(gamepad_2.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                        deliverySystem.armToClip();
                    }

                    break;
                case HIGH_BASKET:

                    deliverySystem.moveBox((double) deliverySystem.getLinear().getCurrentPosition() / linearLevel.getPosition());

                    if (!deliverySystem.getLinear().isBusy()) {
                        if(gamepad_2.wasJustPressed(GamepadKeys.Button.A)) {
                            linearLevel = LinearLevel.GROUND;
                        } else if (gamepad_2.wasJustPressed(GamepadKeys.Button.B)) {
                            linearLevel = LinearLevel.LOW_BASKET;
                        } else if (gamepad_2.wasJustPressed(GamepadKeys.Button.Y)) {
                            linearLevel = LinearLevel.CLIP;
                        }
                    }
                    break;
            }
        }
    }
}