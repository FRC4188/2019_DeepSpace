package robot.commands.drive;

import robot.Robot;
import robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.command.Command;

/** Drives to a given distance in feet using PID loop. */
public class DriveToDistance extends Command {

    Drivetrain drivetrain = Robot.drivetrain;
    
    final double kP = 0;
    final double kI = 0;
    final double kD = 0;

    double lastError, integral = 0;
    double distance, tolerance;

    public DriveToDistance(double distance, double tolerance) {
        requires(Robot.drivetrain);
        this.distance = distance;
        this.tolerance = tolerance;
    }

    @Override
    protected void initialize() {
        // reset fields
        lastError = 0;
        integral = 0;
    }

    @Override
    protected void execute() {
        double input = drivetrain.getPosition();
        double error = distance - input;
        integral += error * drivetrain.DELTA_T;
        double derivative = (error - lastError) / drivetrain.DELTA_T;
        double output = kP * error + kI * integral + kD * derivative;
        lastError = error;
        drivetrain.tank(output, output, 1.0);
    }

    @Override
    protected boolean isFinished() {
        return (Math.abs(lastError) < tolerance);
    }

    @Override
    protected void end() {
        drivetrain.tank(0, 0, 0);
    }

    @Override
    protected void interrupted() {
        end();
    }

}
