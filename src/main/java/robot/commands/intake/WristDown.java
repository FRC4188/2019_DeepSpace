package robot.commands.intake;

import edu.wpi.first.wpilibj.command.Command;
import robot.Robot;
import robot.subsystems.Intake;

/** Pushes the wrist down at a given speed. */
public class WristDown extends Command {

    Intake intake = Robot.intake;
    double speed;

    public WristDown(double speed) {
        requires(intake);
        this.speed = speed;
    }

    @Override
    protected void initialize() {
    }

    @Override
    protected void execute() {
        intake.setWristOpenLoop(-speed);
    }

    @Override
    protected boolean isFinished() {
        return false;
    }

    @Override
    protected void end() {
    }

    @Override
    protected void interrupted() {
    }

}
