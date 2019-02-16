package robot.utils;

import java.util.ArrayList;

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
        return constrain(Math.abs(value), min, max) * sign;
    }

    /** Returns true if specified value is between the min and max, inclusive. */
    public static boolean isBetween(double value, double min, double max) {
        // if out of order, swap
        if(min > max) {
            double tmp_min = min;
            min = max;
            max = tmp_min;
        }
        return (value >= min && value <= max);
    }

    /** Returns the average of an arraylist of numbers */
    public static double average(ArrayList<Double> nums){
        double sum = 0.0;
        for(Double num : nums){
            sum += num;
        }
        return sum / nums.size();
    }
}
