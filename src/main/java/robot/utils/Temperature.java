
package robot.utils;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import robot.Robot;
import robot.subsystems.*;
import robot.utils.CSPMath;
/** Displayed temperatures. */
public class Temperature {

    final int MAX_TEMP = 60; //Cel
    
    Drivetrain drivetrain = Robot.drivetrain;
    Arm arm = Robot.arm;
    Elevator elevator = Robot.elevator;

    double[] temps;

    public void run() {
        
        for(int i = 1; i < 7; i++){ 
            if(drivetrain.getTemperatures(i) > MAX_TEMP){
                SmartDashboard.putString("Temp Warning", "ID "+i+" temp: "+ CSPMath.cToF(drivetrain.getTemperatures(i)) );
            }
        }
        for(int i = 11; i < 13; i++){ 
            if(arm.getTemperatures(i) > MAX_TEMP){
                SmartDashboard.putString("Temp Warning", "ID "+i+" temp: "+ CSPMath.cToF(arm.getTemperatures(i)) );
            }
        }
        for(int i = 21; i < 23; i++){ 
            if(elevator.getTemperatures(i) > MAX_TEMP){
                SmartDashboard.putString("Temp Warning", "ID "+i+" temp: "+ CSPMath.cToF(elevator.getTemperatures(i)) );
            }
        }
    }

}
