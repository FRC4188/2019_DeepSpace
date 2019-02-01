package robot.commands.intake;

import robot.Robot;
import robot.subsystems.Intake;
import edu.wpi.first.wpilibj.command.Command;

/** Sets shoulder to given angle while keeping wrist level. */
public class IntakeWristToAngle extends Command {

    Intake intake = Robot.intake;

    final double kP = 0.1;
    final double kI = 0;
    final double kD = 0;

    double lastError, integral = 0;
    double angle, tolerance;

    public IntakeWristToAngle(double angle, double tolerance) {
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

        // PID loop on intake wrist angle
        double intakeWristInput = intake.getIntakeWristPosition();
        double intakeWristError = angle - intakeWristInput;
        integral += intakeWristError * intake.DELTA_T;
        double derivative = (intakeWristError - lastError) / intake.DELTA_T;
        double intakeWristOutput = kP * intakeWristError + kI * integral + kD * derivative;
        lastError = intakeWristError;

        /*
        // P loop on wrist angle
        double wristInput = arm.getWristPosition();
        double wristError = 0 - wristInput; // drive wrist to 0 degrees
        double wristOutput = kP * wristError;
        */

        // command motor output
        intake.controlWrist(intakeWristOutput, 0.5);

    }

    @Override
    protected boolean isFinished() {
        return (Math.abs(lastError) < tolerance);
    }

    @Override
    protected void end() {
        intake.controlWrist(0, 0);
    }

    @Override
    protected void interrupted() {
        end();
    }

}
