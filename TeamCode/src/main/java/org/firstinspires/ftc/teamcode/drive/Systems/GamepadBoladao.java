package org.firstinspires.ftc.teamcode.drive.Systems;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.Objects;

public class GamepadBoladao extends Gamepad {

    Gamepad mainGamepad;
    Gamepad gamepadOne;
    Gamepad gamepadTwo;

    public GamepadBoladao(Gamepad gamepad) {
        this.mainGamepad = gamepad;
        gamepadOne = new Gamepad();
        gamepadTwo = new Gamepad();
    }

    public void readGamepad(Gamepad gamepad) {
        this.mainGamepad = gamepad;

        if (this.mainGamepad.right_bumper) {
            gamepadOne.reset();
            gamepadTwo.copy(this.mainGamepad);
        } else {
            gamepadTwo.reset();
            gamepadOne.copy(this.mainGamepad);
        }
    }

    public Gamepad getGamepadOne() {
        readGamepad(this.mainGamepad);
        return gamepadOne;
    }

    public Gamepad getGamepadTwo() {
        readGamepad(this.mainGamepad);
        return gamepadTwo;
    }
}
