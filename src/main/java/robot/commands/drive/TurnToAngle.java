package robot.commands.drive;

import robot.Robot;
import robot.subsystems.Drivetrain;
import robot.utils.CSPMath;
import edu.wpi.first.wpilibj.command.Command;

/** Turns to given angle in degrees using PID loop. */
public class TurnToAngle extends Command {

    public enum Angle { RELATIVE, ABSOLUTE }

    Drivetrain drivetrain = Robot.drivetrain;
    
    final double kP = 0.006;
    final double kI = 0;
    final double kD = 0;

    double lastError, integral = 0;
    double angle, tolerance;
    Angle type;

    public TurnToAngle(double angle, double tolerance, Angle type) {
        requires(Robot.drivetrain);
        this.angle = angle;
        this.tolerance = tolerance;
        this.type = type;
    }

    @Override
    protected void initialize() {

        // reset fields
        lastError = 0;
        integral = 0;

        if(type == Angle.RELATIVE) angle += drivetrain.getGyroAngle();
    }

    @Override
    protected void execute() {
        double input = drivetrain.getGyroAngle();
        double error = angle - input;
        integral += error * drivetrain.DELTA_T;
        double derivative = (error - lastError) / drivetrain.DELTA_T;
        double output = kP * error + kI * integral + kD * derivative;
        output = CSPMath.constrainKeepSign(output, 0.15, 1.0);
        lastError = error;
        drivetrain.tank(output, -output, 1.0);
        System.out.println("turning to: " + angle);
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