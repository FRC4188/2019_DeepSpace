package robot.commands.intake;

import robot.Robot;
import robot.subsystems.Intake;
import edu.wpi.first.wpilibj.command.Command;

/** Sets intake wrist to given angle in degrees. */
public class WristToAngle extends Command {

    Intake intake = Robot.intake;

    final double kP = 0.1;
    final double kI = 0;
    final double kD = 0;

    double lastError, integral = 0;
    double angle, tolerance;

    public WristToAngle(double angle, double tolerance) {
        requires(intake);
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
        double input = intake.getWristPosition();
        double error = angle - input;
        integral += error * intake.DELTA_T;
        double derivative = (error - lastError) / intake.DELTA_T;
        double intakeWristOutput = kP * error + kI * integral + kD * derivative;
        lastError = error;
        intake.setWrist(intakeWristOutput);
    }

    @Override
    protected boolean isFinished() {
        return (Math.abs(lastError) < tolerance);
    }

    @Override
    protected void end() {
        intake.setWrist(0);
    }

    @Override
    protected void interrupted() {
        end();
    }

}
