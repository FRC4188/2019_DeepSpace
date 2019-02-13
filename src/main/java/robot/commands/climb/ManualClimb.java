package robot.commands.climb;

import robot.OI;
import robot.Robot;
import robot.subsystems.Climber;
import edu.wpi.first.wpilibj.command.Command;

public class ManualClimb extends Command {

    OI oi = Robot.oi;
    Climber climber = Robot.climber;
    private double percent;

    public ManualClimb(double percent) {
        this.percent = percent;
        requires(climber);
    }

    @Override
    protected void initialize() {
    }

    @Override
    protected void execute() {
        climber.set(percent);
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
