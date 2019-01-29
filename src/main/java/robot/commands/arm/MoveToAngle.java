package robot.commands.arm;

import robot.Robot;
import robot.subsystems.Arm;
import edu.wpi.first.wpilibj.command.Command;

public class MoveToAngle extends Command {
    
    Arm arm = Robot.arm;

    final double kP = 0.1;
    final double kI = 0;
    final double kD = 0;

    double lastError, integral = 0;
    double angle, tolerance;

    public MoveToAngle(double angle, double tolerance) {
        requires(arm);
        this.angle = angle;
        this.tolerance = tolerance;
    }

    @Override
    protected void initialize() {
        lastError = 0;
        integral = 0;
    }

    @Override
    protected void execute() {
        double input = arm.getShoulderPosition(); //Encoder angle
        double error = angle - input;
        integral += error * arm.DELTA_T;
        double shoulderDerivative = (error - lastError) / arm.DELTA_T;
        double output = kP * error + kI * integral + kD * shoulderDerivative;
        lastError = error;
        arm.control(output, output, 1.0);   
    }   

    @Override
    protected boolean isFinished() {
        return (Math.abs(lastError) < tolerance);
    }

    @Override
    protected void end() {
        arm.control(0, 0, 0);
    }

    @Override
    protected void interrupted() {
        end();
    }

}
