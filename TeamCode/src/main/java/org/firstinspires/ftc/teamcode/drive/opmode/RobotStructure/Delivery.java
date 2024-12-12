package org.firstinspires.ftc.teamcode.drive.opmode.RobotStructure;

import static org.firstinspires.ftc.teamcode.drive.opmode.RobotStructure.Delivery.kA;
import static org.firstinspires.ftc.teamcode.drive.opmode.RobotStructure.Delivery.kD;
import static org.firstinspires.ftc.teamcode.drive.opmode.RobotStructure.Delivery.kI;
import static org.firstinspires.ftc.teamcode.drive.opmode.RobotStructure.Delivery.kP;
import static org.firstinspires.ftc.teamcode.drive.opmode.RobotStructure.Delivery.kStatic;
import static org.firstinspires.ftc.teamcode.drive.opmode.RobotStructure.Delivery.kV;
import static org.firstinspires.ftc.teamcode.drive.opmode.RobotStructure.Delivery.MAX_VEL;
import static org.firstinspires.ftc.teamcode.drive.opmode.RobotStructure.Delivery.linear;
import static org.firstinspires.ftc.teamcode.drive.opmode.RobotStructure.Delivery.mp;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

class DeliveryThread implements Runnable {

    @Override
    public void run() {

        double velocity = 0;

        PIDCoefficients coefficients = new PIDCoefficients(kP, kI, kD);
        PIDFController controller = new PIDFController(coefficients, kV, kA, kStatic);

        while(!Thread.currentThread().isInterrupted()) {
            double cp = linear.getCurrentPosition();
            double tp = linear.getTargetPosition();

            if (cp < Math.abs(tp)) {
                velocity = mp.rectFunction(1.0 / 3.0, 0.422);
            } else {
                velocity = 0;
            }
            controller.setOutputBounds(0, MAX_VEL);
            controller.setTargetAcceleration(mp.getAccel());
            controller.setTargetVelocity(velocity);
            controller.setTargetPosition(tp);

            linear.setVelocity(controller.update(cp, linear.getVelocity()));
        }
    }
}

public class Delivery {

    public enum Cycle {
        BASKET,
        CHAMBER,
        IDLE
    }

    public enum Chamber {
        CATCH_SPECIMEN,
        CLIP_SPECIMEN,
        UP_LINEAR,
        IDLE
    }

    volatile public static Chamber chamber = Chamber.IDLE;
    volatile public static Cycle cycle = Cycle.IDLE;

    volatile public static double MAX_VEL = 2800;
    volatile public static double MAX_ACCEL = 2800;

    volatile public static double kP = 0;
    volatile public static double kI = 0;
    volatile public static double kD = 0;

    volatile public static double kV = 0;
    volatile public static double kA = 0;
    volatile public static double kStatic = 0;

    volatile public static MProfile mp;

    HardwareMap hwMap;

    // Motors
    volatile public static DcMotorEx linear;


    // Sensors

    DistanceSensor boxSensor;

    // Servos
    Servo deliveryLeft;
    Servo deliveryRight;
    Servo claw;

    public Delivery(HardwareMap hwMap) {
        this.hwMap = hwMap;

        linear = hwMap.get(DcMotorEx.class, "linear");
        deliveryLeft = hwMap.get(Servo.class, "deliveryLeft");
        deliveryRight = hwMap.get(Servo.class, "deliveryRight");
        claw = hwMap.get(Servo.class, "claw");
        boxSensor = hwMap.get(DistanceSensor.class, "deliverySensor");

        mp = new MProfile(linear, MAX_VEL, MAX_ACCEL);
    }

    /** AUTONOMOUS **/

    ///////// SPECIMEN /////////

    // SIMPLE MOVEMENTS //

    public void closeClaw() {
        claw.setPosition(1.0);
    }
    public void openClaw() {
        claw.setPosition(0.0);
    }
    public void moveLinear(int TARGET_POSITION) {
        linear.setTargetPosition(TARGET_POSITION);
    }
    public void returnLinear() {
        moveLinear(0);
    }
    public void moveBox(double finalPos) {
        deliveryLeft.setPosition(finalPos);
        deliveryRight.setPosition(1 - finalPos);
    }

    // COMPLEX MOVEMENTS //

    public void deliveryBasket() {
        double TARGET_POSITION = 1200;
        moveLinear((int) TARGET_POSITION);

        double path = Math.abs(TARGET_POSITION - linear.getCurrentPosition());

        double p = (100 * path) / TARGET_POSITION;

        moveBox(p / 100.0);
    }

    public void catchSpecimen() {
        if(DriveTrain.canUp) {
            moveLinear(450);
        }
        if(DriveTrain.canCatch) {
            closeClaw();
            moveLinear(650);
        }
    }

    public void clipSpecimen() {
        moveLinear(960);
        openClaw();
    }

    public void deliveryFunctions() {
        switch(cycle) {
            case BASKET:
                break;
            case CHAMBER:

                switch(chamber) {
                    case CATCH_SPECIMEN:
                        catchSpecimen();
                        break;
                    case CLIP_SPECIMEN:
                        clipSpecimen();
                        break;

                    case UP_LINEAR:
                        moveLinear(1100);
                        break;

                    case IDLE:

                        break;
                }

                break;

            case IDLE:

                break;
        }
    }
}
