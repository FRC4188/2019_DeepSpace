package robot.commands.climb;

import robot.OI;
import robot.Robot;
import robot.subsystems.Climber;
import edu.wpi.first.wpilibj.command.Command;

public class ManualClimb extends Command {

    public enum Climbers { FRONT, REAR, ALL }

    OI oi = Robot.oi;
    Climber climber = Robot.climber;

    private double percent;
    Climbers whichClimbers;

    public ManualClimb(double percent, Climbers whichClimbers) {
        this.percent = percent;
        this.whichClimbers = whichClimbers;
    }

    @Override
    protected void initialize() {
    }

    @Override
    protected void execute() {
        if(whichClimbers == Climbers.FRONT) {
            climber.setFront(percent);
        } else if(whichClimbers == Climbers.REAR) {
            climber.setRear(percent);
        } else if(whichClimbers == Climbers.ALL) {
            climber.setFront(percent);
            climber.setRear(percent);
        }
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
