package robot.commands.arm;

import robot.Robot;
import robot.subsystems.Arm;
import edu.wpi.first.wpilibj.command.Command;

/** Sets shoulder to given angle while keeping wrist level. */
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

        // PID loop on shoulder angle
        double shoulderInput = arm.getShoulderPosition();
        double shoulderError = angle - shoulderInput;
        integral += shoulderError * arm.DELTA_T;
        double derivative = (shoulderError - lastError) / arm.DELTA_T;
        double shoulderOutput = kP * shoulderError + kI * integral + kD * derivative;
        lastError = shoulderError;

        // P loop on wrist angle
        double wristInput = arm.getWristPosition();
        double wristError = 0 - wristInput; // drive wrist to 0 degrees
        double wristOutput = kP * wristError;

        // command motor output
        arm.control(shoulderOutput, wristOutput, 1.0);

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
