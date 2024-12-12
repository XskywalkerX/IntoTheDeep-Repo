package org.firstinspires.ftc.teamcode.drive.opmode.RobotStructure;

import static org.firstinspires.ftc.teamcode.drive.opmode.RobotStructure.Intake.MAX_VEL;
import static org.firstinspires.ftc.teamcode.drive.opmode.RobotStructure.Intake.expansion;
import static org.firstinspires.ftc.teamcode.drive.opmode.RobotStructure.Intake.kA;
import static org.firstinspires.ftc.teamcode.drive.opmode.RobotStructure.Intake.kD;
import static org.firstinspires.ftc.teamcode.drive.opmode.RobotStructure.Intake.kI;
import static org.firstinspires.ftc.teamcode.drive.opmode.RobotStructure.Intake.kP;
import static org.firstinspires.ftc.teamcode.drive.opmode.RobotStructure.Intake.kStatic;
import static org.firstinspires.ftc.teamcode.drive.opmode.RobotStructure.Intake.kV;
import static org.firstinspires.ftc.teamcode.drive.opmode.RobotStructure.Intake.mp;

import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


class IntakeThread implements Runnable {

    @Override
    public void run() {

        double velocity = 0;

        PIDCoefficients coefficients = new PIDCoefficients(kP, kI, kD);
        PIDFController controller = new PIDFController(coefficients, kV, kA, kStatic);

        while(!Thread.currentThread().isInterrupted()) {

            double cp = expansion.getCurrentPosition();
            double tp = expansion.getTargetPosition();

            if (cp < Math.abs(tp)) {
                velocity = mp.rectFunction(1.0 / 3.0, 0.422);
            } else {
                velocity = 0;
            }
            controller.setOutputBounds(0, MAX_VEL);
            controller.setTargetAcceleration(mp.getAccel());
            controller.setTargetVelocity(velocity);
            controller.setTargetPosition(tp);

            expansion.setVelocity(controller.update(cp, expansion.getVelocity()));
        }
    }
}
public class Intake {

    enum ExpansionState {
        RETRACTED,
        EXPANDED
    }

    enum ReaperState {
        RUNNING,
        IDLE
    }

    volatile public static double MAX_VEL = 2800;
    volatile public static double MAX_ACCEL = 2800;

    volatile public static double kP = 0;
    volatile public static double kI = 0;
    volatile public static double kD = 0;

    volatile public static double kV = 0;
    volatile public static double kA = 0;
    volatile public static double kStatic = 0;

    volatile public static MProfile mp;

    public static double MULTIPLIER = 0.1;

    HardwareMap hwMap;

    //Motors
    DcMotorEx reaper;
    volatile public static DcMotorEx expansion;

    //Servos
    Servo itkLeft;
    Servo itkRight;


    // Sensors
    DistanceSensor boxSensor;


    volatile public static ExpansionState expansionState = ExpansionState.RETRACTED;
    ReaperState reaperState = ReaperState.IDLE;

    public Intake(HardwareMap hwMap) {
        this.hwMap = hwMap;

        reaper = hwMap.get(DcMotorEx.class, "reaper");
        expansion = hwMap.get(DcMotorEx.class, "expansion");
        itkLeft = hwMap.get(Servo.class, "itkLeft");
        itkRight = hwMap.get(Servo.class, "itkRight");
        boxSensor = hwMap.get(DistanceSensor.class, "itkSensor");

        mp = new MProfile(expansion, MAX_VEL, MAX_ACCEL);
    }

    /** AUTONOMOUS **/

    //////////// SIMPLE MOVEMENTS ////////////

    public void moveBox(double finalPos) {
        itkLeft.setPosition(finalPos);
        itkRight.setPosition(1 - finalPos);
    }

    public void downBox() {
        moveBox(1.0);
    }

    public void upBox() {
        moveBox(0.0);
    }

    public void moveExpansion(int TARGET_POSITION) {
        expansion.setTargetPosition(TARGET_POSITION);
    }

    public void forwardExpansion() {
        moveExpansion(700);
    }

    public void returnExpansion() {
        moveExpansion(0);
        upBox();
    }


    ///////////// COMPLEX MOVEMENTS /////////////

    public void catchSample(double sampleWidth) {

        double TARGET_POSITION = sampleWidth * MULTIPLIER;
        moveExpansion((int) TARGET_POSITION);

        double path = Math.abs(TARGET_POSITION - expansion.getCurrentPosition());

        double p = (100 * path) / TARGET_POSITION;

        moveBox(p / 100.0);
    }

    ////////////// STATE //////////////

    public void intakeFunctions(double sampleWidth) {
        switch(expansionState) {
            case RETRACTED:

                reaperState = ReaperState.IDLE;
                returnExpansion();

                break;
            case EXPANDED:

                reaperState = ReaperState.RUNNING;
                catchSample(sampleWidth);

                if(boxSensor.getDistance(DistanceUnit.CM) < 5.0) {
                    expansionState = ExpansionState.RETRACTED;
                }

                break;
        }

        switch (reaperState) {
            case RUNNING:
                reaper.setPower(0.7);
                break;
            case IDLE:
                reaper.setPower(0.0);
                break;
        }
    }
}
