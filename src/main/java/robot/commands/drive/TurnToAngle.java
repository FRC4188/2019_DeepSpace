package robot.commands.drive;

import robot.Robot;
import robot.subsystems.Drivetrain;
import robot.utils.CSPMath;
import edu.wpi.first.wpilibj.command.Command;
import jaci.pathfinder.Pathfinder;

/** Turns to given angle in degrees using PID loop. */
public class TurnToAngle extends Command {

    public enum Angle { RELATIVE, ABSOLUTE }

    Drivetrain drivetrain = Robot.drivetrain;

    final double kP = 0.01;
    final double kI = 0;
    final double kD = 0;
    final double DELTA_T = 0.02; // seconds

    double lastError, integral = 0;
    double angle, tolerance, angleParam, counter;
    Angle type;

    public TurnToAngle(double angle, double tolerance, Angle type) {
        requires(Robot.drivetrain);
        setName("TurnToAngle " + type.toString() + ": " + angle);
        this.angleParam = angle;
        this.tolerance = tolerance;
        this.type = type;
    }

    @Override
    protected void initialize() {

        // determine setpoint based on type
        if(type == Angle.RELATIVE) angle = angleParam + drivetrain.getGyroAngle();
        else angle = angleParam;

        // reset fields and ensure angle is in range
        lastError = integral = counter = 0;
        angle = Pathfinder.boundHalfDegrees(angle);

    }

    @Override
    protected void execute() {
        double input = drivetrain.getGyroAngle();
        double error = angle - input;
        if(Math.abs(error) > 180) error = (error > 0) ? error - 360 : error + 360; // take shortest path to angle
        integral += error * DELTA_T;
        double derivative = (error - lastError) / DELTA_T;
        double output = kP * error + kI * integral + kD * derivative;
        output = CSPMath.constrainKeepSign(output, 0.21, 1.0);
        lastError = error;
        drivetrain.tank(output, -output, 1.0);
        if(Math.abs(error) < tolerance) counter++;
        else counter = 0;
    }

    @Override
    protected boolean isFinished() {
        return counter > 5;
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
