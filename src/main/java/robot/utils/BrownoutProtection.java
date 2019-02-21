package robot.utils;

import edu.wpi.first.wpilibj.PowerDistributionPanel;

/** Motitors battery voltage and provides variable to scale down motor output. */
public class BrownoutProtection {

    private PowerDistributionPanel pdp = new PowerDistributionPanel();
    double brownoutVar = 1.0;
    final double MIN_VOLTAGE = 7.2;

    /** Reads battery voltage and adjusts brownout var accordingly. */
    public void run() {
        if(pdp.getVoltage() < MIN_VOLTAGE) brownoutVar = 0.7;
        else brownoutVar = 1.0;
    }

    /** Returns current brownout variable based on battery voltage. */
    public double getBrownoutVar() {
        return brownoutVar;
    }

}

