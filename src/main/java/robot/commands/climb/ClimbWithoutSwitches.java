package robot.commands.climb;

import robot.Robot;
import robot.subsystems.Climber;
import edu.wpi.first.wpilibj.command.Command;

/** Sets climber to a given speed without observing limit
 *  switch state. Positive value extends. */
public class ClimbWithoutSwitches extends Command {

    Climber climber = Robot.climber;
    double percent;

    public ClimbWithoutSwitches(double percent) {
        this.percent = percent;
    }

    @Override
    protected void initialize() {
    }

    @Override
    protected void execute() {
        climber.setLeft(percent);
        climber.setRight(percent);
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
