package robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import robot.commands.drive.FollowLine;

public class OI {

    private class Controller {
        final static int A = 1;
        final static int B = 2;
        final static int X = 3;
        final static int Y = 4;
        final static int LB = 5;
        final static int RB = 6;
        final static int BACK = 7;
        final static int START = 8;
        final static int LS = 9;
        final static int RS = 10;
        final static double DEADBAND = 0.1;
    }

    private class RocketBox {
        final static int GROUND = 1;
        final static int CARGO_LOW = 2;
        final static int CARGO_MID = 3;
        final static int CARGO_HIGH = 4;
        final static int HATCH_LOW = 5;
        final static int HATCH_MID = 6;
        final static int HATCH_HIGH = 7;
    }

    // Controller initialization
    private XboxController pilot = new XboxController(0);
    private XboxController coPilot = new XboxController(1);
    private Joystick rocketBox = new Joystick(3);

    // Button initialization
    private JoystickButton pilotA = new JoystickButton(pilot, Controller.A);
    private JoystickButton pilotB = new JoystickButton(pilot, Controller.B);
    private JoystickButton pilotX = new JoystickButton(pilot, Controller.X);
    private JoystickButton pilotY = new JoystickButton(pilot, Controller.Y);
    private JoystickButton pilotLb = new JoystickButton(pilot, Controller.LB);
    private JoystickButton pilotRb = new JoystickButton(pilot, Controller.RB);
    private JoystickButton pilotBack = new JoystickButton(pilot, Controller.BACK);
    private JoystickButton pilotStart = new JoystickButton(pilot, Controller.START);
    private JoystickButton pilotLs = new JoystickButton(pilot, Controller.LS);
    private JoystickButton pilotRs = new JoystickButton(pilot, Controller.RS);

    private JoystickButton coPilotA = new JoystickButton(coPilot, Controller.A);
    private JoystickButton coPilotB = new JoystickButton(coPilot, Controller.B);
    private JoystickButton coPilotX = new JoystickButton(coPilot, Controller.X);
    private JoystickButton coPilotY = new JoystickButton(coPilot, Controller.Y);
    private JoystickButton coPilotLb = new JoystickButton(coPilot, Controller.LS);
    private JoystickButton coPilotRb = new JoystickButton(coPilot, Controller.RB);
    private JoystickButton coPilotBack = new JoystickButton(coPilot, Controller.BACK);
    private JoystickButton coPilotStart = new JoystickButton(coPilot, Controller.START);
    private JoystickButton coPilotLs = new JoystickButton(coPilot, Controller.LS);
    private JoystickButton coPilotRs = new JoystickButton(coPilot, Controller.RS);

    private JoystickButton rbGround = new JoystickButton(rocketBox, RocketBox.GROUND);
    private JoystickButton rbCargoLow = new JoystickButton(rocketBox, RocketBox.CARGO_LOW);
    private JoystickButton rbCargoMid = new JoystickButton(rocketBox, RocketBox.CARGO_MID);
    private JoystickButton rbCargoHigh = new JoystickButton(rocketBox, RocketBox.CARGO_HIGH);
    private JoystickButton rbHatchLow = new JoystickButton(rocketBox, RocketBox.HATCH_LOW);
    private JoystickButton rbHatchMid = new JoystickButton(rocketBox, RocketBox.HATCH_MID);
    private JoystickButton rbHatchHigh = new JoystickButton(rocketBox, RocketBox.HATCH_HIGH);

    /** Constructs new OI object and assigns commands. */
    public OI() {
        pilotA.whenPressed(new FollowLine());
    }

    /** Returns y axis of Joystick on pilot controller. */
    public double getPilotY(Hand hand) {
        if(Math.abs(pilot.getY(hand)) > Controller.DEADBAND) return 0;
        else return -pilot.getY(hand);
    }

    /** Returns x axis of Joystick on pilot controller. */
    public double getPilotX(Hand hand) {
        if(Math.abs(pilot.getX(hand)) > Controller.DEADBAND) return 0;
        else return pilot.getX(hand);
    }

    /** Returns y axis of Joystick on coPilot controller. */
    public double getCoPilotY(Hand hand) {
        if(Math.abs(coPilot.getY(hand)) > Controller.DEADBAND) return 0;
        else return -coPilot.getY(hand);
    }

    /** Returns x axis of Joystick on coPilot controller. */
    public double getCoPilotX(Hand hand) {
        if(Math.abs(coPilot.getX(hand)) > Controller.DEADBAND) return 0;
        else return coPilot.getX(hand);
    }

}
