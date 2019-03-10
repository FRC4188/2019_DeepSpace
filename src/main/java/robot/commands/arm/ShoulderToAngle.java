package robot.commands.arm;

import robot.Robot;
import robot.subsystems.Arm;
import edu.wpi.first.wpilibj.command.Command;

/** Sets shoulder on arm to given angle in degrees. */
public class ShoulderToAngle extends Command {

    Arm arm = Robot.arm;
    double angle, tolerance, counter;

    public ShoulderToAngle(double angle, double tolerance) {
        requires(arm);
        setName("ShoulderToAngle: " + angle);
        this.angle = angle;
        this.tolerance = tolerance;
    }

    @Override
    protected void initialize() {
        counter = 0;
    }

    @Override
    protected void execute() {
        arm.shoulderToAngle(angle, tolerance);
        double error = angle - arm.getPosition();
        if(Math.abs(error) < tolerance) counter++;
        else counter = 0;
    }

    @Override
    protected boolean isFinished() {
        return counter > 5;
    }

    @Override
    protected void end() {
        arm.set(0);
    }

    @Override
    protected void interrupted() {
        end();
    }

}
