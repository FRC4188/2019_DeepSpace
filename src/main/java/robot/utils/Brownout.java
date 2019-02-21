package robot.utils;

import edu.wpi.first.wpilibj.PowerDistributionPanel;
import robot.subsystems.Arm;
import robot.subsystems.Climber;
import robot.subsystems.Drivetrain;
import robot.subsystems.Elevator;
import robot.subsystems.Intake;

public class Brownout {

    // Created a constants and objects//
    public double brownoutVariable;
    public static PowerDistributionPanel powerDistributionPanel;

    public enum VoltageState {
        NORMAL, CONSERVING
    }

    public static VoltageState voltageState = VoltageState.NORMAL;

    // Initialize objects//
    public static void init() {
        powerDistributionPanel = new PowerDistributionPanel();
    }

    // Create the battery//
    public static void isBrownout() {
        //Arm Brownout//
        if (voltageState == voltageState.NORMAL) {
            if (powerDistributionPanel.getVoltage() < 7.0) {
                Arm.conservePower(true);
                voltageState = VoltageState.CONSERVING;
            }
        }
        if (voltageState == voltageState.CONSERVING) {
	       		Arm.conservePower(false);
	        	voltageState = VoltageState.NORMAL;
        }
        
       //Climber Brownout//
        if (voltageState == voltageState.NORMAL) {
            if (powerDistributionPanel.getVoltage() < 7.0) {
                Climber.conservePower(true);
                voltageState = VoltageState.CONSERVING;
            }
        }
        if (voltageState == voltageState.CONSERVING) {
	       		Climber.conservePower(false);
	        	voltageState = VoltageState.NORMAL;
        }

       //Drivetrain Brownout//
        if (voltageState == voltageState.NORMAL) {
            if (powerDistributionPanel.getVoltage() < 7.0) {
                Drivetrain.conservePower(true);
                voltageState = VoltageState.CONSERVING;
            }
        }
        if (voltageState == voltageState.CONSERVING) {
	       		Drivetrain.conservePower(false);
	        	voltageState = VoltageState.NORMAL;
            }
        
       //Elevator Brownout//
        if (voltageState == voltageState.NORMAL) {
            if (powerDistributionPanel.getVoltage() < 7.0) {
                Elevator.conservePower(true);
                voltageState = VoltageState.CONSERVING;
            }
        }
        if (voltageState == voltageState.CONSERVING) {
	       		Elevator.conservePower(false);
	        	voltageState = VoltageState.NORMAL;
        }

       //Intake Brownout//
        if (voltageState == voltageState.NORMAL) {
            if (powerDistributionPanel.getVoltage() < 7.0) {
                Intake.conservePower(true);
                voltageState = VoltageState.CONSERVING;
            }
        }
        if (voltageState == voltageState.CONSERVING) {
	       		Intake.conservePower(false);
	        	voltageState = VoltageState.NORMAL;
        }
        
    }
}
