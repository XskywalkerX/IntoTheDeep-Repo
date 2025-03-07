package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.drive.Systems.Vision.SamplePipeline;
import org.firstinspires.ftc.teamcode.enums.ClawStates;
import org.firstinspires.ftc.teamcode.enums.DeliveryStates;
import org.firstinspires.ftc.teamcode.enums.IntakeStates;
import org.firstinspires.ftc.teamcode.systems.ArmIntakeSystem;
import org.firstinspires.ftc.teamcode.systems.DeliveryClaw;
import org.firstinspires.ftc.teamcode.systems.DeliverySystem;
import org.firstinspires.ftc.teamcode.systems.GamepadBoladao;
import org.firstinspires.ftc.teamcode.systems.HorizontalLinear;
import org.firstinspires.ftc.teamcode.systems.IntakeClaw;
import org.firstinspires.ftc.teamcode.systems.IntakeSystem;
import org.firstinspires.ftc.teamcode.systems.VerticalLinear;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.List;

@TeleOp(name = "BACK IN BLACK")
public class TeleOpLoop extends LinearOpMode {

    GamepadBoladao gamepadBoladao;

    IntakeSystem intakeSystem;
    DeliverySystem deliverySystem;

    Robot robot;

    OpenCvWebcam webcam;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new Robot(hardwareMap);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        dashboard.startCameraStream(webcam, 30);

        gamepadBoladao = new GamepadBoladao(gamepad1);
        intakeSystem = new IntakeSystem();
        deliverySystem = new DeliverySystem();

        robot.leftLinear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightLinear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftLinear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightLinear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        waitForStart();
        while (opModeIsActive()) {
            double horizontalMultiplier = gamepad1.right_trigger - gamepad1.left_trigger;
            // Read the current list
            List<ColorBlobLocatorProcessor.Blob> blobs = robot.colorLocator.getBlobs();
            ColorBlobLocatorProcessor.Util.filterByArea(1500, 500000, blobs);  // filter out very small blobs.
            gamepadBoladao.readGamepad(gamepad1);
            intakeSystem.update(robot, horizontalMultiplier, blobs);
            deliverySystem.update(robot);


            if (gamepadBoladao.ONEwasYPressed()) {
                IntakeSystem.CS = IntakeStates.IDLE;
            }
            if (gamepadBoladao.ONEwasBPressed() && IntakeSystem.CS != IntakeStates.READING) {
                IntakeSystem.CS = IntakeStates.READING;
            } else if (gamepadBoladao.ONEwasBPressed() && IntakeSystem.CS == IntakeStates.READING) {
                IntakeSystem.CS = IntakeStates.CATCH;
            }
            if (gamepadBoladao.ONEwasAPressed()) {
                IntakeSystem.CS = IntakeStates.DROP;
            }

            if (gamepadBoladao.TWOwasYPressed()) {
                IntakeSystem.CS = IntakeStates.IDLE;
                DeliverySystem.CS = DeliveryStates.TRANSFER;
            }
            if (gamepadBoladao.TWOwasAPressed() && DeliverySystem.CS == DeliveryStates.TRANSFER) {
                DeliverySystem.CS = DeliveryStates.SPECIMEN_OUTTAKE_1;
                IntakeSystem.CS = IntakeStates.IDLE;
            }
            if (gamepadBoladao.TWOwasBPressed()) {
                DeliverySystem.CS = DeliveryStates.SPECIMEN_INTAKE;
                IntakeSystem.CS = IntakeStates.IDLE;
            }
            if (gamepadBoladao.TWOwasXPressed() && DeliverySystem.CS == DeliveryStates.TRANSFER) {
                DeliverySystem.CS = DeliveryStates.HIGH_BASKET;
                IntakeSystem.CS = IntakeStates.IDLE;
            }

            if (gamepadBoladao.TWOwasLeftBumperPressed()) {
                if (DeliveryClaw.CS == ClawStates.OPENED) {
                    DeliveryClaw.CS = ClawStates.CLOSED;
                } else {
                    DeliveryClaw.CS = ClawStates.OPENED;
                }
            }
            if (gamepadBoladao.ONEwasLeftBumperPressed() && ( IntakeSystem.CS == IntakeStates.DROP )) {
                if (IntakeClaw.CS == ClawStates.OPENED) {
                    IntakeClaw.CS = ClawStates.CLOSED;
                } else {
                    IntakeClaw.CS = ClawStates.OPENED;
                }
            }


            telemetry.addLine("============== INTAKE SYSTEM ==============");
            telemetry.addLine();
            telemetry.addData("INTAKE STATE", IntakeSystem.CS.name());
            telemetry.addData("INTAKE CLAW STATE", IntakeClaw.CS.name());
            telemetry.addData("ARM INTAKE STATE", ArmIntakeSystem.CS.name());
            telemetry.addData("HORIZONTAL LINEAR STATE", HorizontalLinear.CS.name());
            telemetry.addLine();
            telemetry.addLine();
            telemetry.addLine("============== DELIVERY SYSTEM ==============");
            telemetry.addLine();
            telemetry.addData("DELIVERY STATE", DeliverySystem.CS.name());
            telemetry.addData("DELIVERY CLAW STATE", DeliveryClaw.CS.name());
            telemetry.addData("VERTICAL LINEAR STATE", VerticalLinear.CS.name());
            telemetry.update();
        }
    }
}