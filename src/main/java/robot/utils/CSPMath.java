package robot.utils;

/** Contains various math related utilities. */
public class CSPMath {

    /** Constrains value to specified bounds. */
    public static double constrain(double value, double min, double max) {
        if(value > max) value = max;
        else if(value < min) value = min;
        return value;
    }

    /** Constrains absolute value of value to specified bounds 
     *  and returns value with original sign. */
    public static double constrainKeepSign(double value, double min, double max) {
        double sign = Math.signum(value);
        return constrain(value, min, max) * sign;
    }

}
