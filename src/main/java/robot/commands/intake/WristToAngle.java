package robot.commands.intake;

import robot.Robot;
import robot.subsystems.Intake;
import edu.wpi.first.wpilibj.command.Command;

/** Sets intake wrist to given angle in degrees. */
public class WristToAngle extends Command {

    Intake intake = Robot.intake;
    double angle, tolerance, counter;

    public WristToAngle(double angle, double tolerance) {
        requires(intake);
        setName("WristToAngle: " + angle);
        this.angle = angle;
        this.tolerance = tolerance;
    }

    @Override
    protected void initialize() {
        counter = 0;
    }

    @Override
    protected void execute() {
        intake.wristToAngle(angle, tolerance);
        double error = angle - intake.getWristPosition();
        if(Math.abs(error) < tolerance) counter ++;
        else counter = 0;
    }

    @Override
    protected boolean isFinished() {
        return counter > 5;
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
