
package robot.utils;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import robot.Robot;
import robot.subsystems.*;
import robot.utils.CSPMath;

/** Display temperatures of motors accessed by CAN ID. */
public class Temperature {

    final int MAX_TEMP = 60; //Cel
    
    Drivetrain drivetrain = Robot.drivetrain;
    Arm arm = Robot.arm;
    Elevator elevator = Robot.elevator;

    /** Gets temperature of every motor and writes to Smart Dash in F if temp is over 60 C*/
    public void run() {
        
        for(int i = 1; i < 7; i++){ 
            if(drivetrain.getMotorTemperature(i) > MAX_TEMP){
                SmartDashboard.putString("Temp Warning", "ID "+i+" temp: "+ CSPMath.cToF(drivetrain.getMotorTemperature(i)) );
            }
        }
        for(int i = 11; i < 13; i++){ 
            if(arm.getMotorTemperature(i) > MAX_TEMP){
                SmartDashboard.putString("Temp Warning", "ID "+i+" temp: "+ CSPMath.cToF(arm.getMotorTemperature(i)) );
            }
        }
        for(int i = 21; i < 23; i++){ 
            if(elevator.getMotorTemperature(i) > MAX_TEMP){
                SmartDashboard.putString("Temp Warning", "ID "+i+" temp: "+ CSPMath.cToF(elevator.getMotorTemperature(i)) );
            }
        }
    }

}
