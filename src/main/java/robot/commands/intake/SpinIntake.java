package robot.commands.intake;

import robot.Robot;
import robot.subsystems.Intake;
import edu.wpi.first.wpilibj.command.Command;

/** Spins cargo intake to percent speed (-1.0, 1.0) in given direction. */
public class SpinIntake extends Command {

    Intake intake = Robot.intake;

    double speed;

    public SpinIntake(double speed) {
        requires(intake);
        this.speed = speed;
    }

    @Override
    protected void initialize() {
    }

    @Override
    protected void execute() {
        intake.spinIntake(speed);
        intake.setWristOpenLoop(speed);
    }

    @Override
    protected boolean isFinished() {
        return true;
    }

    @Override
    protected void end() {
    }

    @Override
    protected void interrupted() {
    }

}
