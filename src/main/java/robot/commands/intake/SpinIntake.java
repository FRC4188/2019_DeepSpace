package robot.commands.intake;

import robot.OI;
import robot.Robot;
import robot.subsystems.Intake;
import edu.wpi.first.wpilibj.command.Command;

/** Spins cargo intake to percent speed (-1.0, 1.0) in given direction. */
public class SpinIntake extends Command {

    public enum Direction { IN, OUT }

    OI oi = Robot.oi;
    Intake intake = Robot.intake;

    double speed;
    Direction direction;

    public SpinIntake(double speed, Direction direction) {
        requires(intake);
        this.speed = speed;
        this.direction = direction;
    }

    @Override
    protected void initialize() {
        if(direction == Direction.IN) speed = -speed;
    }

    @Override
    protected void execute() {
        intake.spinIntake(speed);
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
