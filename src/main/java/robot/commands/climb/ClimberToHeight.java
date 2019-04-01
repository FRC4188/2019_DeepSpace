package robot.commands.climb;

import robot.Robot;
import robot.subsystems.Climber;
import edu.wpi.first.wpilibj.command.Command;

/** Sets climbers to a given height in feet. */
public class ClimberToHeight extends Command {

    Climber climber = Robot.climber;
    double height, tolerance, counter;

    public ClimberToHeight(double height, double tolerance) {
        requires(climber);
        setName("ClimberToHeight: " + height);
        this.height = height;
        this.tolerance = tolerance;
    }

    @Override
    protected void initialize() {
        counter = 0;
    }

    @Override
    protected void execute() {
        climber.climberToHeight(height, tolerance);
        double error = height - climber.getPosition();
        if(Math.abs(error) < tolerance) counter++;
        else counter = 0;
    }

    @Override
    protected boolean isFinished() {
        return counter > 10;
    }

    @Override
    protected void end() {
        climber.setRightOpenLoop(0);
        climber.setLeftOpenLoop(0);
    }

    @Override
    protected void interrupted() {
        end();
    }

}
