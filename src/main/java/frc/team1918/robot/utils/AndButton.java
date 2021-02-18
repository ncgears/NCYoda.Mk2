package frc.team1918.robot.utils;

import edu.wpi.first.wpilibj2.command.button.Button;

/**
 * This class creates a Button object that requires 2 buttons to be pressed at the same time.
 */
public class AndButton extends Button {
    private final Button button1;
    private final Button button2;

    public AndButton(Button buttonOne, Button buttonTwo) {
        button1 = buttonOne;
        button2 = buttonTwo;
    }

    @Override
    public boolean get() {
        return button1.get() && button2.get();
    }
}