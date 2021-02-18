package frc.team1918.robot.utils;

import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.POVButton;

public class OrPOVButton extends Button {
    private final POVButton button1;
    private final POVButton button2;
    private final POVButton button3;

    public OrPOVButton(POVButton buttonOne, POVButton buttonTwo, POVButton buttonThree) {
        button1 = buttonOne;
        button2 = buttonTwo;
        button3 = buttonThree;
    }

    @Override
    public boolean get() {
        return button1.get() || button2.get() || button3.get();
    }
}