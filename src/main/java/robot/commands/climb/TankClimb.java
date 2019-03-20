package robot.commands.climb;

import robot.Robot;
import robot.subsystems.Climber;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;

/** Runs climber motors individually using pilot triggers. */
public class TankClimb extends Command {

    Climber climber = Robot.climber;

    public TankClimb() {
        requires(climber);
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

        double leftPercent = Robot.oi.getPilotTrigger(Hand.kLeft);
        double rightPercent = Robot.oi.getPilotTrigger(Hand.kRight);
        double leftOutput = 0;
        double rightOutput = 0;
        if(leftPercent > 0 && leftCanExtend) leftOutput = leftPercent;
        if(rightPercent > 0 && rightCanExtend) rightOutput = rightPercent;
        if(leftPercent < 0 && leftCanRetract) leftOutput = leftPercent;
        if(rightPercent < 0 && rightCanRetract) rightOutput = rightPercent;

        // command motor output
        climber.setLeft(leftOutput);
        climber.setRight(rightOutput);

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
