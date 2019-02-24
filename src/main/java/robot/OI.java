package robot;

import robot.commands.drive.*;
import robot.commands.drive.FollowPath.Path;
import robot.commands.drive.ShiftGear.Gear;
import robot.commands.groups.*;
import robot.commands.groups.ToHeight.Height;
import robot.commands.drive.FollowObject.Object;
import robot.commands.intake.*;
import robot.commands.arm.*;
import robot.commands.climb.*;
import robot.utils.*;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.buttons.POVButton;
import edu.wpi.first.wpilibj.buttons.Trigger;

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
        public final static int DPAD_OFF = -1;
        public final static int DPAD_NORTH = 0;
        public final static int DPAD_NE = 45;
        public final static int DPAD_EAST = 90;
        public final static int DPAD_SE = 135;
        public final static int DPAD_SOUTH = 180;
        public final static int DPAD_SW = 225;
        public final static int DPAD_WEST = 270;
        public final static int DPAD_NW = 315;
        public final static double DEADBAND = 0.08;
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
    private Joystick rocketBox = new Joystick(2);

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
    private POVButton pilotDpadNorth = new POVButton(pilot, Controller.DPAD_NORTH);
    private POVButton pilotDpadEast = new POVButton(pilot, Controller.DPAD_EAST);
    private POVButton pilotDpadSouth = new POVButton(pilot, Controller.DPAD_SOUTH);
    private POVButton pilotDpadWest = new POVButton(pilot, Controller.DPAD_WEST);

    private JoystickButton copilotA = new JoystickButton(copilot, Controller.A);
    private JoystickButton copilotB = new JoystickButton(copilot, Controller.B);
    private JoystickButton copilotX = new JoystickButton(copilot, Controller.X);
    private JoystickButton copilotY = new JoystickButton(copilot, Controller.Y);
    private JoystickButton copilotLb = new JoystickButton(copilot, Controller.LB);
    private JoystickButton copilotRb = new JoystickButton(copilot, Controller.RB);
    private JoystickButton copilotBack = new JoystickButton(copilot, Controller.BACK);
    private JoystickButton copilotStart = new JoystickButton(copilot, Controller.START);
    private JoystickButton copilotLS = new JoystickButton(copilot, Controller.LS);
    private JoystickButton copilotRS = new JoystickButton(copilot, Controller.RS);
    private POVButton copilotDpadNorth = new POVButton(copilot, Controller.DPAD_NORTH);
    private POVButton copilotDpadEast = new POVButton(copilot, Controller.DPAD_EAST);
    private POVButton copilotDpadSouth = new POVButton(copilot, Controller.DPAD_SOUTH);
    private POVButton copilotDpadWest = new POVButton(copilot, Controller.DPAD_WEST);

    private JoystickButton rbGround = new JoystickButton(rocketBox, RocketBox.GROUND);
    private JoystickButton rbCargoLow = new JoystickButton(rocketBox, RocketBox.CARGO_LOW);
    private JoystickButton rbCargoMid = new JoystickButton(rocketBox, RocketBox.CARGO_MID);
    private JoystickButton rbCargoHigh = new JoystickButton(rocketBox, RocketBox.CARGO_HIGH);
    private JoystickButton rbHatchLow = new JoystickButton(rocketBox, RocketBox.HATCH_LOW);
    private JoystickButton rbHatchMid = new JoystickButton(rocketBox, RocketBox.HATCH_MID);
    private JoystickButton rbHatchHigh = new JoystickButton(rocketBox, RocketBox.HATCH_HIGH);

    // Iterator initialization
    private Trigger copilotLTrig = new TriggerAsButton(copilot, Hand.kLeft);
    private Trigger copilotRTrig = new TriggerAsButton(copilot, Hand.kRight);
    private CommandIterator hatchIterator = new CommandIterator((Trigger) copilotLb, copilotLTrig, 160, "Hatch Iterator");
    private CommandIterator cargoIterator = new CommandIterator((Trigger) copilotRb, copilotRTrig, 160, "Cargo Iterator");

    /** Constructs new OI object and assigns commands. */
    public OI() {

        pilotLS.whenPressed(new ShiftGear(Gear.HIGH));
        pilotLS.whenReleased(new ShiftGear(Gear.OFF));
        pilotRS.whenPressed(new ShiftGear(Gear.LOW));
        pilotRS.whenPressed(new ShiftGear(Gear.OFF));

        pilotB.whenPressed(new FollowLine());
        pilotX.whenPressed(new FollowObject(Object.BAY_CLOSE));
        pilotY.whenPressed(new FollowPath(Path.TO_PERPENDICULAR, false));

        pilotDpadNorth.whileHeld(new ManualClimb(0.2));
        pilotDpadNorth.whenReleased(new ManualClimb(0));
        pilotDpadSouth.whileHeld(new ManualClimb(-0.2));
        pilotDpadSouth.whenReleased(new ManualClimb(0));

        pilotBack.whenPressed(new KillAll());
        copilotBack.whenPressed(new KillAll());

        copilotA.whileHeld(new SpinIntake(1.0));
        copilotA.whenReleased(new SpinIntake(0));
        copilotB.whileHeld(new SpinIntake(-1.0));
        copilotB.whenReleased(new SpinIntake(0));

        copilotY.whenPressed(new FireHatch(Value.kForward));
        copilotY.whenReleased(new FireHatch(Value.kOff));
        copilotX.whenPressed(new FireHatch(Value.kReverse));
        copilotX.whenReleased(new FireHatch(Value.kOff));

        hatchIterator.runCmdWhenValue(new ToHeight(Height.HATCH_LOW), 1);
        hatchIterator.runCmdWhenValue(new ToHeight(Height.HATCH_MID), 2);
        hatchIterator.runCmdWhenValue(new ToHeight(Height.HATCH_HIGH), 3);
        hatchIterator.start();

        cargoIterator.runCmdWhenValue(new ToHeight(Height.CARGO_LOW), 1);
        cargoIterator.runCmdWhenValue(new ToHeight(Height.CARGO_MID), 2);
        cargoIterator.runCmdWhenValue(new ToHeight(Height.CARGO_HIGH), 3);
        cargoIterator.start();

    }

    // options to scale joystick input
    private enum JoystickSens {
        LINEAR,
        SQUARED,
        CUBED,
        TESSERACTED,
        SINE
    }

    /** Returns value scaled to proper sensitivity based on current JoystickSens. */
    private double scaleJoystick(double val, JoystickSens sens) {
        if(sens == JoystickSens.LINEAR) return val;
        else if(sens == JoystickSens.SQUARED) return Math.signum(val) * Math.pow(val, 2);
        else if(sens == JoystickSens.CUBED) return Math.pow(val, 3);
        else if(sens == JoystickSens.TESSERACTED) return Math.signum(val) * Math.pow(val, 4);
        else if(sens == JoystickSens.SINE) return Math.sin(val * (Math.PI / 2));
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
        else return scaleJoystick(pilot.getX(hand), JoystickSens.SINE);
    }

    /** Returns trigger axis on pilot controller. */
    public double getPilotTrigger(Hand hand) {
        return pilot.getTriggerAxis(hand);
    }

    /** Returns the value of the Dpad on pilot controller (multiples of 45).
     *  0 is up, 90 right, etc. -1.0 if not currently pressed. */
    public double getPilotDpad() {
        return pilot.getPOV();
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

    /** Returns trigger axis on copilot controller. */
    public double getCopilotTrigger(Hand hand) {
        return copilot.getTriggerAxis(hand);
    }

    /** Returns the value of the Dpad on copilot controller (multiples of 45).
     *  0 is up, 90 right, etc. -1.0 if not currently pressed. */
    public double getCopilotDpad() {
        return copilot.getPOV();
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
