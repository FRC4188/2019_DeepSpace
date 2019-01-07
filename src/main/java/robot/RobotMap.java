package robot;

public class RobotMap {
   
    // Robot constants
    public static final double MAX_VELOCITY = 0; // ft/s
    public static final double MAX_ACCELERATION = 0; // ft/s^2
    public static final double MAX_JERK = 0; // ft/s^3
    public static final double WHEELBASE_WIDTH = 0; // ft
    public static final double WHEEL_DIAMETER = (0.0 / 12.0); // ft
    public static final int TICKS_PER_REV = 4096; // talon units
    public static final double RAMP_RATE = 2; // seconds
    
    // Drive talon ID's
    public static final int LEFT = 0;
    public static final int LEFT_SLAVE1 = 0;
    public static final int LEFT_SLAVE2 = 0;
    public static final int RIGHT = 0;
    public static final int RIGHT_SLAVE1 = 0;
    public static final int RIGHT_SLAVE2 = 0;

}
