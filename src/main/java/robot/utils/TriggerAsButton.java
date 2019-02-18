package robot.utils;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.buttons.Trigger;

/** Allows trigger axis on controller to act as a button. */
public class TriggerAsButton extends Trigger {

    private Hand hand;
    private XboxController joystick;
    private double threshold;

    /** Constructs new object that will poll a trigger on a given XboxController
     *  on the given hand to see if it exceeds a default threshold of 0.6. */
    public TriggerAsButton(XboxController joystick, Hand hand) {
        this.joystick = joystick;
        this.hand = hand;
        this.threshold = 0.6;
    }

    /** Constructs new object that will poll a trigger on a given XboxController
     *  on the given hand to see if it exceeds a given threshold. */
    public TriggerAsButton(XboxController joystick, Hand hand, double threshold) {
        this.joystick = joystick;
        this.hand = hand;
        this.threshold = threshold;
    }

    public boolean get() {
        return joystick.getTriggerAxis(hand) > threshold;
    }

}
