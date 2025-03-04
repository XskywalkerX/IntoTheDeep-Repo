package org.firstinspires.ftc.teamcode.RobotStructure;

import com.qualcomm.robotcore.hardware.Gamepad;

public class GamepadControl {

    Gamepad movementDriver;
    Gamepad driverTwo;

    public GamepadControl() {

        movementDriver = new Gamepad();
        driverTwo = new Gamepad();
    }

    public void setColor(double r, double g, double b) {

        movementDriver.setLedColor(r, g, b, Gamepad.LED_DURATION_CONTINUOUS);
        driverTwo.setLedColor(r, g, b, Gamepad.LED_DURATION_CONTINUOUS);
    }

    public void setColorEasy(String color) {

        if(color.equalsIgnoreCase("GREEN")) {
            movementDriver.setLedColor(0, 255, 0, Gamepad.LED_DURATION_CONTINUOUS);
            driverTwo.setLedColor(0, 255, 0, Gamepad.LED_DURATION_CONTINUOUS);

        } else if(color.equalsIgnoreCase("BLUE")) {
            movementDriver.setLedColor(0, 0, 255, Gamepad.LED_DURATION_CONTINUOUS);
            driverTwo.setLedColor(0, 0, 255, Gamepad.LED_DURATION_CONTINUOUS);

        } else if(color.equalsIgnoreCase("RED")) {
            movementDriver.setLedColor(255, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);
            driverTwo.setLedColor(255, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);
        } else if(color.equalsIgnoreCase("YELLOW")) {
            movementDriver.setLedColor(255, 255, 0, Gamepad.LED_DURATION_CONTINUOUS);
            driverTwo.setLedColor(255, 255, 0, Gamepad.LED_DURATION_CONTINUOUS);

        } else if(color.equalsIgnoreCase("PURPLE")) {
            movementDriver.setLedColor(255, 0, 255, Gamepad.LED_DURATION_CONTINUOUS);
            driverTwo.setLedColor(255, 0, 255, Gamepad.LED_DURATION_CONTINUOUS);

        } else {
            movementDriver.setLedColor(255, 255, 255, Gamepad.LED_DURATION_CONTINUOUS);
            driverTwo.setLedColor(255, 255, 255, Gamepad.LED_DURATION_CONTINUOUS);
        }
    }

    public void blip(int times) {
        movementDriver.rumbleBlips(times);
        driverTwo.rumbleBlips(times);
    }

    public void rumble(int duration) {
        movementDriver.rumble(duration);
        driverTwo.rumble(duration);
    }

    public Gamepad getMovementDriver() {
        return movementDriver;
    }

    public Gamepad getDriverTwo() {
        return driverTwo;
    }
}
