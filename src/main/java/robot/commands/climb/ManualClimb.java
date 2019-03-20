package robot.commands.climb;

import robot.Robot;
import robot.subsystems.Climber;
import edu.wpi.first.wpilibj.command.Command;

/** Runs climber motors at a given percent.
 *  Positive percent extends. */
public class ManualClimb extends Command {

    Climber climber = Robot.climber;
    double percent;

    public ManualClimb(double percent) {
        requires(climber);
        this.percent = percent;
    }

    @Override
    protected void initialize() {
    }

    @Override
    protected void execute() {

        // handle limit switches
        boolean leftTopSwitch = climber.getLeftTopSwitch();
        boolean rightTopSwitch = climber.getRightTopSwitch();
        boolean leftBottomSwitch = climber.getLeftBottomSwitch();
        boolean rightBottomSwitch = climber.getRightBottomSwitch();
        boolean leftCanExtend, rightCanExtend, leftCanRetract, rightCanRetract;
        if(leftTopSwitch) { // top
            leftCanExtend = true;
            leftCanRetract = false;
        } else if(leftBottomSwitch) { // bottom
            leftCanExtend = false;
            leftCanRetract = true;
        } else { // middle
            leftCanExtend = true;
            leftCanRetract = true;
        }
        if(rightTopSwitch) { // top
            rightCanExtend = true;
            rightCanRetract = false;
        } else if(rightBottomSwitch) { // bottom
            rightCanExtend = false;
            rightCanRetract = true;
        } else { // middle
            rightCanExtend = true;
            rightCanRetract = true;
        }

        double leftPercent = 0;
        double rightPercent = 0;
        if(percent > 0) {
            if(leftCanExtend) leftPercent = percent;
            if(rightCanExtend) rightPercent = percent;
        } else if(percent < 0) {
            if(leftCanRetract) leftPercent = percent;
            if(rightCanRetract) rightPercent = percent;
        }

        // command motor output
        climber.setLeft(leftPercent);
        climber.setRight(rightPercent);

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
