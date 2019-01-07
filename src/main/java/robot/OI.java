package robot; 

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

public class OI {

    class Mappings {
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
    }

    // Controller declarations
    public static XboxController pilot;
    public static XboxController coPilot;

    // Button declarations
    public JoystickButton pilotA, pilotB, pilotX, pilotY, pilotLb, pilotRb,
            pilotBack, pilotStart, pilotLs, pilotRs;
    public JoystickButton coPilotA, coPilotB, coPilotX, coPilotY, coPilotLb, coPilotRb,
            coPilotBack, coPilotStart, coPilotLs, coPilotRs;

    public OI() {

        // Controller initialization
        pilot = new XboxController(0);
        coPilot = new XboxController(1);

        // Button initialization
        pilotA = new JoystickButton(pilot, Mappings.A);
        pilotB = new JoystickButton(pilot, Mappings.B);
        pilotX = new JoystickButton(pilot, Mappings.X);
        pilotY = new JoystickButton(pilot, Mappings.Y);
        pilotLb = new JoystickButton(pilot, Mappings.LB);
        pilotRb = new JoystickButton(pilot, Mappings.RB);
        pilotBack = new JoystickButton(pilot, Mappings.BACK);
        pilotStart = new JoystickButton(pilot, Mappings.START);
        pilotLs = new JoystickButton(pilot, Mappings.LS);
        pilotRs = new JoystickButton(pilot, Mappings.RS);
        
        coPilotA = new JoystickButton(pilot, Mappings.A);
        coPilotB = new JoystickButton(pilot, Mappings.B);
        coPilotX = new JoystickButton(pilot, Mappings.X);
        coPilotY = new JoystickButton(pilot, Mappings.Y);
        coPilotLb = new JoystickButton(pilot, Mappings.LB);
        coPilotRb = new JoystickButton(pilot, Mappings.RB);
        coPilotBack = new JoystickButton(pilot, Mappings.BACK);
        coPilotStart = new JoystickButton(pilot, Mappings.START);
        coPilotLs = new JoystickButton(pilot, Mappings.LS);
        coPilotRs = new JoystickButton(pilot, Mappings.RS);

        // Command mappings
        // on the way
        
    }

}
