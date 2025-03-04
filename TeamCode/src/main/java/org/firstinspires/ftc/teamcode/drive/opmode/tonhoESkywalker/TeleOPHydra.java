package org.firstinspires.ftc.teamcode.tonhoESkywalker;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RR.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RobotStructure.PIDController.PIDController;

@Config
@TeleOp(name = "TELE OP HYDRA X XMACHINE", group = "aSTART")
@Disabled
public class TeleOPHydra extends LinearOpMode {
    private Servo bracao; // PORT 3 CONTROL
    private Servo garra; // PORT 2 CONTROL
    private Servo giratorio; // PORT 1 CONTROL
    private CRServo intakeLeft; // PORT 4 CONTROL
    private CRServo intakeRIght; // PORT 5 CONTROL

    private DcMotorEx linear; // PORT 0 CONTROL
    private DcMotorEx reaper; // PORT X CONTROL

    public static double bracaoCollect = 0.7;
    public static double bracaoDelivery = 0.7;

    public static double giratorioCollect = 0.9;
    public static double giratorioDelivery = 0.2;

    public static double garraClose = 1;
    public static double garraOpen = 0;

    public static int linearLevelOne = 200;
    public static int linearLevelTwo = 600;
    public static int linearLevelThree = 800;

    public static double linearKp = 2;
    public static double linearKi = 0;
    public static double linearKd = 0;


    @Override
    public void runOpMode() throws InterruptedException {

        bracao = hardwareMap.get(Servo.class, "bracao");
        garra = hardwareMap.get(Servo.class, "garra");
        giratorio = hardwareMap.get(Servo.class, "giratorio");
        intakeLeft = hardwareMap.get(CRServo.class, "intakeLeft");
        intakeRIght = hardwareMap.get(CRServo.class, "intakeRight");

        linear = hardwareMap.get(DcMotorEx.class, "linear");
        reaper = hardwareMap.get(DcMotorEx.class, "reaper");

        linear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        PIDController linearPID = new PIDController(linearKp, linearKi, linearKd);


        waitForStart();

        while (opModeIsActive()) {
            linearPID.setKp(linearKp);
            linearPID.setKd(linearKd);
            linearPID.setKi(linearKi);

            linearPID.setTargetPosition(linear.getTargetPosition());
            linearPID.setCurrentPosition(linear.getCurrentPosition());
            linear.setPower(linearPID.dice());

            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            (gamepad1.right_stick_y - gamepad1.right_stick_x)
                    )
            );

            if (gamepad2.b) { // coleta
                linear.setTargetPosition(linearLevelOne);
                bracao.setPosition(bracaoCollect);
                giratorio.setPosition(giratorioCollect);
            }

            if (gamepad2.x) {
                linear.setTargetPosition(linearLevelTwo);
                bracao.setPosition(bracaoDelivery);
                giratorio.setPosition(giratorioDelivery);
            }

            if (gamepad2.y) {
                linear.setTargetPosition(linearLevelThree);
            }


            if (gamepad2.right_trigger != 0) {
                garra.setPosition(garraOpen);
            } else {
                garra.setPosition(garraClose);
            }



            telemetry.addData("bracao position: ", bracao.getPosition());
            telemetry.addData("giratorio position: ", giratorio.getPosition());
            telemetry.addData("garra position: ", garra.getPosition());
            telemetry.update();
        }
    }
}
