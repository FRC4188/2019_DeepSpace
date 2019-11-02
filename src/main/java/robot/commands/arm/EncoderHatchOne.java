package robot.commands.arm;
 
import edu.wpi.first.wpilibj.command.Command;
import robot.subsystems.Arm;
import robot.Robot;
import robot.subsystems.Elevator;
import robot.subsystems.Intake;
 
public class EncoderHatchOne extends Command {
 
 Arm arm = Robot.arm;
 Elevator elevator = Robot.elevator;
 Intake intake = Robot.intake;
 
 public EncoderHatchOne() {
 }
 
 // Called just before this Command runs the first time
 @Override
 protected void initialize() {
 }
 
 // Called repeatedly when this Command is scheduled to run
 @Override
 protected void execute() {
   
   elevator.setEncoders(2.34);
   arm.setEncoders(-115.9);
   intake.setEncoders(-37);
   
 }
 
 // Make this return true when this Command no longer needs to run execute()
 @Override
 protected boolean isFinished() {
   return true;
 }
 
 // Called once after isFinished returns true
 @Override
 protected void end() {
 }
 
 // Called when another command which requires one or more of the same
 // subsystems is scheduled to run
 @Override
 protected void interrupted() {
 }
}
