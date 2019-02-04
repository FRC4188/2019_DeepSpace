package robot.commands.drive;

import robot.Robot;
import robot.subsystems.Drivetrain;
import robot.subsystems.LimeLight;
import edu.wpi.first.wpilibj.command.Command;

/** Drives to a given distance in feet using PID loop. */
public class DriveToDistance extends Command {

    public enum Distance { RELATIVE, ABSOLUTE, PERP_LENGTH }

    Drivetrain drivetrain = Robot.drivetrain;
    LimeLight limelight = Robot.limelight;

    final double kP = 0.1;
    final double kI = 0;
    final double kD = 0;

    double lastError, integral = 0;
    double distance, tolerance, distanceParam;
    Distance type;

    public DriveToDistance(double distance, double tolerance, Distance type) {
        requires(drivetrain);
        this.distanceParam = distance;
        this.tolerance = tolerance;
        this.type = type;
    }

    /** Drives necessary length to reach perpendicular point of target
     *  if type is set to PERP_LENGTH. Otherwise, do nothing. */
    public DriveToDistance(Distance type) {
        requires(drivetrain);
        requires(limelight);
        this.tolerance = 0.5;
        this.type = type;
    }

    @Override
    protected void initialize() {

        // determine setpoint based on type
        if(type == Distance.RELATIVE) distance = distanceParam + drivetrain.getPosition();
        if(type == Distance.PERP_LENGTH) distance = limelight.solvePerpendicular()[1];
        else distance = distanceParam;

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
