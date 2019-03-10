package robot.commands.drive;

import robot.Robot;
import robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.command.Command;

/** Drives to a given distance in feet using PID loop.
 *  All distances relative to starting position. */
public class DriveToDistance extends Command {

    Drivetrain drivetrain = Robot.drivetrain;
    double distance, tolerance, counter, initialDist;

    public DriveToDistance(double distance, double tolerance) {
        requires(drivetrain);
        setName("DriveToDistance: " + distance);
        this.distance = distance;
        this.tolerance = tolerance;
    }

    @Override
    protected void initialize() {
        counter = 0;
        initialDist = drivetrain.getPosition();
        distance += initialDist;
    }

    @Override
    protected void execute() {
        drivetrain.driveToDistance(distance, tolerance);
        double relativePos = drivetrain.getPosition() + initialDist;
        double error = distance - relativePos;
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
