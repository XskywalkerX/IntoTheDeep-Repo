package org.firstinspires.ftc.teamcode.drive.Systems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class IntakeSystem {

    Servo intakeX, intakeZ;
    DcMotorEx expansion;

    public IntakeSystem(HardwareMap hwMap) {

        intakeX = hwMap.get(Servo.class, "itkX");
        intakeZ = hwMap.get(Servo.class, "itkZ");

        expansion = hwMap.get(DcMotorEx.class, "expansion");

        expansion.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}