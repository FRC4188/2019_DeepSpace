package robot;

import robot.commands.drive.*;
import robot.commands.drive.FollowObject.Object;
import robot.commands.drive.TurnToAngle.Angle;
import robot.commands.groups.DepositToBay;
import robot.utils.KillAll;
import robot.utils.Paths;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

public class OI {

    // button mappings for logitech controller
    public class Controller {
        public final static int A = 1;
        public final static int B = 2;
        public final static int X = 3;
        public final static int Y = 4;
        public final static int LB = 5;
        public final static int RB = 6;
        public final static int BACK = 7;
        public final static int START = 8;
        public final static int LS = 9;
        public final static int RS = 10;
        public final static double DEADBAND = 0.1;
    }

    // button mappings for rocket box
    public class RocketBox {
        public final static int GROUND = 1;
        public final static int CARGO_LOW = 2;
        public final static int CARGO_MID = 3;
        public final static int CARGO_HIGH = 4;
        public final static int HATCH_LOW = 5;
        public final static int HATCH_MID = 6;
        public final static int HATCH_HIGH = 7;
    }

    // Controller initialization
    private XboxController pilot = new XboxController(0);
    private XboxController copilot = new XboxController(1);
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
    private JoystickButton pilotLS = new JoystickButton(pilot, Controller.LS);
    private JoystickButton pilotRS = new JoystickButton(pilot, Controller.RS);

    private JoystickButton copilotA = new JoystickButton(copilot, Controller.A);
    private JoystickButton copilotB = new JoystickButton(copilot, Controller.B);
    private JoystickButton copilotX = new JoystickButton(copilot, Controller.X);
    private JoystickButton copilotY = new JoystickButton(copilot, Controller.Y);
    private JoystickButton copilotLb = new JoystickButton(copilot, Controller.LS);
    private JoystickButton copilotRb = new JoystickButton(copilot, Controller.RB);
    private JoystickButton copilotBack = new JoystickButton(copilot, Controller.BACK);
    private JoystickButton copilotStart = new JoystickButton(copilot, Controller.START);
    private JoystickButton copilotLS = new JoystickButton(copilot, Controller.LS);
    private JoystickButton copilotRS = new JoystickButton(copilot, Controller.RS);

    private JoystickButton rbGround = new JoystickButton(rocketBox, RocketBox.GROUND);
    private JoystickButton rbCargoLow = new JoystickButton(rocketBox, RocketBox.CARGO_LOW);
    private JoystickButton rbCargoMid = new JoystickButton(rocketBox, RocketBox.CARGO_MID);
    private JoystickButton rbCargoHigh = new JoystickButton(rocketBox, RocketBox.CARGO_HIGH);
    private JoystickButton rbHatchLow = new JoystickButton(rocketBox, RocketBox.HATCH_LOW);
    private JoystickButton rbHatchMid = new JoystickButton(rocketBox, RocketBox.HATCH_MID);
    private JoystickButton rbHatchHigh = new JoystickButton(rocketBox, RocketBox.HATCH_HIGH);

    /** Constructs new OI object and assigns commands. */
    public OI() {
        pilotA.whenPressed(new FollowPath(Paths.testPath, false));
        pilotB.whenPressed(new TurnToAngle(27, 5, Angle.RELATIVE));
        pilotX.whenPressed(new FollowObject(Object.BAY_HIGH));
        pilotY.whenPressed(new DepositToBay());
        pilotLS.whenPressed(new ShiftGear());
        pilotBack.whenPressed(new KillAll());
        copilotBack.whenPressed(new KillAll());
    }

    // options to scale joystick input
    private enum JoystickSens {
        LINEAR, // most sensitive
        SQUARED,
        CUBED,
        TESSERACTED // least sensitive
    }

    /** Returns value scaled to proper sensitivity based on current JoystickSens. */
    private double scaleJoystick(double val, JoystickSens sens) {
        if(sens == JoystickSens.LINEAR) return val;
        else if(sens == JoystickSens.SQUARED) return Math.signum(val) * Math.pow(val, 2);
        else if(sens == JoystickSens.CUBED) return Math.pow(val, 3);
        else if(sens == JoystickSens.TESSERACTED) return Math.signum(val) * Math.pow(val, 4);
        else return val;
    }

    /** Returns y axis of Joystick on pilot controller. */
    public double getPilotY(Hand hand) {
        if(Math.abs(pilot.getY(hand)) < Controller.DEADBAND) return 0;
        else return scaleJoystick(-pilot.getY(hand), JoystickSens.SQUARED);
    }

    /** Returns x axis of Joystick on pilot controller. */
    public double getPilotX(Hand hand) {
        if(Math.abs(pilot.getX(hand)) < Controller.DEADBAND) return 0;
        else return scaleJoystick(pilot.getX(hand), JoystickSens.SQUARED);
    }

    /** Returns trigger axis on pilot controller. */
    public double getPilotTrigger(Hand hand) {
        return pilot.getTriggerAxis(hand);
    }

    /** Returns y axis of Joystick on copilot controller. */
    public double getCopilotY(Hand hand) {
        if(Math.abs(copilot.getY(hand)) < Controller.DEADBAND) return 0;
        else return scaleJoystick(-copilot.getY(hand), JoystickSens.SQUARED);
    }

    /** Returns x axis of Joystick on copilot controller. */
    public double getCopilotX(Hand hand) {
        if(Math.abs(copilot.getX(hand)) < Controller.DEADBAND) return 0;
        else return scaleJoystick(copilot.getX(hand), JoystickSens.SQUARED);
    }
    
    /** Returns trigger axis on pilot controller. */
    public double getCopilotTrigger(Hand hand) {
        return copilot.getTriggerAxis(hand);
    }

    /** Returns state of given button on pilot controller. */
    public boolean getPilotButton(int button) {
        return pilot.getRawButton(button);
    }

    /** Returns state of given button on copilot controller. */
    public boolean getCopilotButton(int button) {
        return copilot.getRawButton(button);
    }

    /** Returns state of given button on the rocket box. */
    public boolean getRocketBoxButton(int button) {
        return rocketBox.getRawButton(button);
    }

}
