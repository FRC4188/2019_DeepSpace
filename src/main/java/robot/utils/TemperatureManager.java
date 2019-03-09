
package robot.utils;

import robot.Robot;
import robot.subsystems.*;
import robot.utils.CSPMath;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Display temperatures of motors accessed by CAN ID. */
public class TemperatureManager {

    final int MAX_TEMP = 55; //Cel

    Drivetrain drivetrain = Robot.drivetrain;
    Arm arm = Robot.arm;
    Elevator elevator = Robot.elevator;

    /** Gets temperature of every motor and writes to Smart Dash in F if temp is over a max temp. */
    public void run() {
        StringBuilder sb = new StringBuilder();
        for(int i = 1; i < 7; i++){
            if(drivetrain.getMotorTemperature(i) > MAX_TEMP){
                double tempInF = CSPMath.cToF(drivetrain.getMotorTemperature(i));
                sb.append("D" + i + ": " + tempInF + ", ");
            }
        }
        for(int i = 11; i < 13; i++){
            if(elevator.getMotorTemperature(i) > MAX_TEMP){
                double tempInF = CSPMath.cToF(elevator.getMotorTemperature(i));
                sb.append("E" + i + ": " + tempInF + ", ");
            }
        }
        for(int i = 21; i < 23; i++){
            if(arm.getMotorTemperature(i) > MAX_TEMP){
                double tempInF = CSPMath.cToF(arm.getMotorTemperature(i));
                sb.append("A" + i + ": " + tempInF + ", ");
            }
        }
        SmartDashboard.putString("Temp Warnings", sb.toString());
    }

}
