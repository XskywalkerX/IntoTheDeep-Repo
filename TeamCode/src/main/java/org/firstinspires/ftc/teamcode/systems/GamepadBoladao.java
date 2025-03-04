package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.hardware.Gamepad;
import java.util.HashMap;
import java.util.Map;

public class GamepadBoladao {
    private final Gamepad mainGamepad;
    private final Gamepad gamepadOne;
    private final Gamepad gamepadTwo;

    private final Map<String, Boolean> currentState = new HashMap<>();
    private final Map<String, Boolean> previousState = new HashMap<>();

    private final String[] buttonsToTrack = {"b", "y", "a", "x", "left_bumper", "dpad_up", "dpad_down"}; // Add more buttons if needed

    public GamepadBoladao(Gamepad gamepad) {
        this.mainGamepad = gamepad;
        this.gamepadOne = new Gamepad();
        this.gamepadTwo = new Gamepad();

        // Initialize button states
        for (String button : buttonsToTrack) {
            currentState.put(button + "_ONE", false);
            previousState.put(button + "_ONE", false);
            currentState.put(button + "_TWO", false);
            previousState.put(button + "_TWO", false);
        }
    }

    private boolean wasButtonPressed(String button, String gamepadId) {
        return currentState.get(button + "_" + gamepadId) && !previousState.get(button + "_" + gamepadId);
    }

    public boolean ONEwasBPressed() { return wasButtonPressed("b", "ONE"); }
    public boolean TWOwasBPressed() { return wasButtonPressed("b", "TWO"); }
    public boolean ONEwasYPressed() { return wasButtonPressed("y", "ONE"); }
    public boolean TWOwasYPressed() { return wasButtonPressed("y", "TWO"); }
    public boolean ONEwasAPressed() { return wasButtonPressed("a", "ONE"); }
    public boolean TWOwasAPressed() { return wasButtonPressed("a", "TWO"); }
    public boolean ONEwasXPressed() { return wasButtonPressed("x", "ONE"); }
    public boolean TWOwasXPressed() { return wasButtonPressed("x", "TWO"); }
    public boolean ONEwasLeftBumperPressed() { return wasButtonPressed("left_bumper", "ONE"); }
    public boolean TWOwasLeftBumperPressed() { return wasButtonPressed("left_bumper", "TWO"); }
    public boolean ONEwasDpadUpPressed() { return wasButtonPressed("dpad_up", "ONE"); }
    public boolean TWOwasDpadUpPressed() { return wasButtonPressed("dpad_up", "TWO"); }
    public boolean ONEwasDpadDownPressed() { return wasButtonPressed("dpad_up", "ONE"); }
    public boolean TWOwasDpadDOwnPressed() { return wasButtonPressed("dpad_down", "TWO"); }


    public void readGamepad(Gamepad gamepad) {
        // Update previous state
        for (String button : buttonsToTrack) {
            previousState.put(button + "_ONE", currentState.get(button + "_ONE"));
            previousState.put(button + "_TWO", currentState.get(button + "_TWO"));
        }

        if (gamepad.right_bumper) {
            gamepadOne.reset();
            gamepadTwo.copy(gamepad);
        } else {
            gamepadTwo.reset();
            gamepadOne.copy(gamepad);
        }

        // Update current state
        for (String button : buttonsToTrack) {
            boolean buttonPressedOne = getButtonState(button, gamepadOne);
            boolean buttonPressedTwo = getButtonState(button, gamepadTwo);

            currentState.put(button + "_ONE", buttonPressedOne);
            currentState.put(button + "_TWO", buttonPressedTwo);
        }
    }

    private boolean getButtonState(String button, Gamepad gamepad) {
        switch (button) {
            case "a": return gamepad.a;
            case "b": return gamepad.b;
            case "x": return gamepad.x;
            case "y": return gamepad.y;
            case "left_bumper": return gamepad.left_bumper;
            case "right_bumper": return gamepad.right_bumper;
            case "dpad_up": return gamepad.dpad_up;
            case "dpad_down": return gamepad.dpad_down;
            default: return false;
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
