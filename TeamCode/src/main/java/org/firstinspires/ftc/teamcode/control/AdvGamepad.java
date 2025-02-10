package org.firstinspires.ftc.teamcode.control;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.HashMap;
import java.util.Map;

import kotlin.Pair;
import kotlin.Triple;

public class AdvGamepad
{
    ///// Create Gamepad Variables
    // Gamepad States
    public Gamepad currentGamepad1 = new Gamepad();   // This is to prevent rapid
    public Gamepad currentGamepad2 = new Gamepad();   // toggling of gamepad inputs.
    public Gamepad previousGamepad1 = new Gamepad();  // Holding down a button no
    public Gamepad previousGamepad2 = new Gamepad();  // longer repeats inputs.
    // Gamepad Inputs
    public enum GamepadInput {left_stick_x, left_stick_y, right_stick_x, right_stick_y,
                              left_trigger, right_trigger, left_bumper, right_bumper,
                              a, b, x, y, dpad_up, dpad_down, dpad_left, dpad_right,
                              start, back}
    public enum InputType {onPress, onButtonHold, onRelease}
    // Gamepad Actions
    public final Map<Triple<Integer, GamepadInput, InputType>, Runnable> gamepadActions = new HashMap<>();
    public final Map<Triple<Integer, GamepadInput, InputType>, Pair<Boolean, Runnable>> constrainedActions = new HashMap<>();
    /////

    public AdvGamepad(Gamepad gamepad1, Gamepad gamepad2)
    {
        updateGamepad(gamepad1, gamepad2);
    }

    public void updateGamepad(Gamepad gamepad1, Gamepad gamepad2)
    {
        // Update gamepad states every loop
        previousGamepad1.copy(currentGamepad1);
        previousGamepad2.copy(currentGamepad2);
        currentGamepad1.copy(gamepad1);
        currentGamepad2.copy(gamepad2);

        // Check for gamepad input and execute corresponding actions
        for (Map.Entry<Triple<Integer, GamepadInput, InputType>, Runnable> entry : gamepadActions.entrySet()) {
            int gamepadNumber = entry.getKey().getFirst();
            GamepadInput input = entry.getKey().getSecond();
            InputType type = entry.getKey().getThird();
            Runnable action = entry.getValue();

            if (type == InputType.onPress) {
                if (onPress(gamepadNumber, input)) {
                    action.run();
                }
            } else if (type == InputType.onButtonHold) {
                if (onButtonHold(gamepadNumber, input)) {
                    action.run();
                }
                if (trigger(gamepadNumber, input) != 0) {
                    action.run();
                }
            }
        }
        for (Map.Entry<Triple<Integer, GamepadInput, InputType>, Pair<Boolean, Runnable>> entry : constrainedActions.entrySet()) {
            if (entry.getValue().getFirst()) {
                int gamepadNumber = entry.getKey().getFirst();
                GamepadInput input = entry.getKey().getSecond();
                InputType type = entry.getKey().getThird();
                Runnable action = entry.getValue().getSecond();

                if (type == InputType.onPress) {
                    if (onPress(gamepadNumber, input)) {
                        action.run();
                    }
                } else if (type == InputType.onButtonHold) {
                    if (onButtonHold(gamepadNumber, input)) {
                        action.run();
                    }
                    if (trigger(gamepadNumber, input) != 0) {
                        action.run();
                    }
                }
            }
        }
    }

    public void addAction(Integer gamepadNumber, GamepadInput input, InputType type, Runnable action)
    {
        Triple<Integer, GamepadInput, InputType> triple = new Triple<>(gamepadNumber, input, type);
        gamepadActions.put(triple, action);
    }

    public void addAction(Integer gamepadNumber, GamepadInput input, InputType type, Boolean constraint, Runnable action)
    {
        Triple<Integer, GamepadInput, InputType> triple = new Triple<>(gamepadNumber, input, type);
        Pair<Boolean, Runnable> pair = new Pair<>(constraint, action);
        constrainedActions.put(triple, pair);
    }

    public boolean onPress(int gamepadNumber, GamepadInput input) {
        Gamepad current = (gamepadNumber == 1) ? currentGamepad1 : currentGamepad2;
        Gamepad previous = (gamepadNumber == 1) ? previousGamepad1 : previousGamepad2;

        // Check if the input is a button
        if (input.ordinal() >= GamepadInput.left_bumper.ordinal() &&
                input.ordinal() <= GamepadInput.back.ordinal()) {
            try {
                // Use reflection to get the button value
                boolean currentValue = (boolean) current.getClass().getField(input.name()).get(current);
                boolean previousValue = (boolean) previous.getClass().getField(input.name()).get(previous);
                return currentValue && !previousValue;
            } catch (Exception e) {
                return false; // Handle exceptions (e.g., field not found)
            }
        } else {
            // Handle triggers & joysticks (assume they are not "pressed" like buttons)
            return false;
        }
    }

    public boolean onButtonHold(int gamepadNumber, GamepadInput input) {
        Gamepad current = (gamepadNumber == 1) ? currentGamepad1 : currentGamepad2;

        // Check if the input is a button
        if (input.ordinal() >= GamepadInput.left_bumper.ordinal() &&
                input.ordinal() <= GamepadInput.back.ordinal()) {
            try {
                // Use reflection to get the button value

                return (boolean) current.getClass().getField(input.name()).get(current);
            } catch (Exception e) {
                return false; // Handle exceptions (e.g., field not found)
            }
        } else {
            // Handle triggers & joysticks (assume they are not "pressed" like buttons)
            return false;
        }
    }

    public double trigger(int gamepadNumber, GamepadInput input) {
        Gamepad current = (gamepadNumber == 1) ? currentGamepad1 : currentGamepad2;

         // Check if the input is a trigger
        if (input.ordinal() == GamepadInput.left_trigger.ordinal() ||
                input.ordinal() == GamepadInput.right_trigger.ordinal()) {
            try {
                // Use reflection to get the trigger value
                return (double) current.getClass().getField(input.name()).get(current);
            } catch (Exception e) {
                return 0; // Handle exceptions (e.g., field not found)
            }
        } else {
            return 0;
        }
    }
}
