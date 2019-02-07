package robot.commands.arm;

import robot.Robot;
import robot.subsystems.Arm;
import edu.wpi.first.wpilibj.command.Command;

/** Sets shoulder on arm to given angle in degrees. */
public class ShoulderToAngle extends Command {

    Arm arm = Robot.arm;

    final double kP = 0.1;
    final double kI = 0;
    final double kD = 0;

    double lastError, integral = 0;
    double angle, tolerance;

    public ShoulderToAngle(double angle, double tolerance) {
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
        double input = arm.getPosition();
        double error = angle - input;
        integral += error * arm.DELTA_T;
        double derivative = (error - lastError) / arm.DELTA_T;
        double output = kP * error + kI * integral + kD * derivative;
        lastError = error;
        arm.set(output);
    }

    @Override
    protected boolean isFinished() {
        return (Math.abs(lastError) < tolerance);
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
