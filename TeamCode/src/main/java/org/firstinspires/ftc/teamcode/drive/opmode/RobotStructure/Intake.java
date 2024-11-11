package org.firstinspires.ftc.teamcode.drive.opmode.RobotStructure;

import static org.firstinspires.ftc.teamcode.drive.opmode.RobotStructure.Pipeline.blueArea;
import static org.firstinspires.ftc.teamcode.drive.opmode.RobotStructure.Pipeline.redArea;
import static org.firstinspires.ftc.teamcode.drive.opmode.RobotStructure.Pipeline.yellowArea;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.opmode.RobotStructure.PIDController.PIDController;

public class Intake {

    public static double CONTROL = 1;
    public static double DISTANCE = 5;

    public static double kp = 0;
    public static double ki = 0;
    public static double kd = 0;

    volatile public static boolean UP = false;

    volatile public static boolean blueSide = false;
    volatile public static boolean hasSample = false;

    volatile public static boolean expanded = false;

    public static double MULTIPLIER = 0.1;
    public static int CATCH_COLOR = 0;
    public static int CATCH_YELLOW = 0;
    public static int CLOSED_ITK = 0;

    DcMotorEx expansion;
    DcMotorEx itk;
    Servo HERE;
    DistanceSensor itkSensor;

    PIDController controller;

    HardwareMap hwMap;

    public Intake(HardwareMap hwMap) {

        this.hwMap = hwMap;

        expansion = hwMap.get(DcMotorEx.class, "expansion");
        itk = hwMap.get(DcMotorEx.class, "itk");
        HERE = hwMap.get(Servo.class, "here");
        itkSensor = hwMap.get(DistanceSensor.class, "itkSensor");

        itk.setDirection(DcMotorSimple.Direction.REVERSE);

        HERE.scaleRange(0.5, 1.0);

        controller = new PIDController(kp, ki, kd);
    }

    public void update() {
        if (UP) {
            controller.setTargetPosition(expansion.getTargetPosition());
            controller.setCurrentPosition(expansion.getCurrentPosition());
            expansion.setPower(controller.dice() / CONTROL);
        }

        hasSample = itkSensor.getDistance(DistanceUnit.CM) <= DISTANCE;
    }


    //movements
    public void expand(boolean blueSide) {
        CATCH_COLOR = blueSide ? (int) (blueArea * MULTIPLIER) : (int) (redArea * MULTIPLIER);
        CATCH_YELLOW = (int) (yellowArea * MULTIPLIER);
        getExpansion().setTargetPosition(Math.min(CATCH_COLOR, CATCH_YELLOW));
    }

    public void catchYellow() {
        CATCH_YELLOW = (int) (yellowArea * MULTIPLIER);
        getExpansion().setTargetPosition(Math.min(CATCH_COLOR, CATCH_YELLOW));
    }

    public void spin() {
        itk.setPower(hasSample ? 0 : 1);
    }

    public void closeExpansion() {
        getExpansion().setTargetPosition(CLOSED_ITK);
    }

    public void downClaw() {
        HERE.setPosition(1);
    }

    public void upClaw() {
        HERE.setPosition(0);
    }


    //complex movements
    public void catchSample() {
        catchYellow();
        if (getExpansion().getCurrentPosition() >= getExpansion().getTargetPosition() * 0.25) {
            downClaw();
            spin();
        }
    }

    public void catchTeleOp() {
        expand(blueSide);
        if (getExpansion().getCurrentPosition() >= getExpansion().getTargetPosition() * 0.25) {
            downClaw();
            spin();
        }
    }

    public void returnIntake() {
        upClaw();
        closeExpansion();
    }


    //get
    public DcMotorEx getExpansion() {
        return expansion;
    }

    public DcMotorEx getItk() {
        return itk;
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("INTAKE POWER", expansion.getPower());
        telemetry.addData("INTAKE TARGET", expansion.getTargetPosition());
        telemetry.addData("INTAKE POSITION", expansion.getCurrentPosition());
        telemetry.update();
    }
}
