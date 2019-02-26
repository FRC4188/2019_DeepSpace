package robot.commands.climb;

import robot.OI;
import robot.Robot;
import robot.subsystems.Climber;
import edu.wpi.first.wpilibj.command.Command;

/** Runs climber using pilot Dpad (up extends). */
public class ManualClimb extends Command {

    OI oi = Robot.oi;
    Climber climber = Robot.climber;

    double percent;
    boolean leftCanExtend, rightCanExtend;
    boolean leftCanRetract, rightCanRetract;
    boolean lastLeftSwitch, lastRightSwitch;
    double lastLeftSpeed, lastRightSpeed;

    public ManualClimb() {
        requires(climber);
    }

    @Override
    protected void initialize() {
        leftCanExtend = rightCanExtend = true;
        leftCanRetract = rightCanRetract = false;
        lastLeftSwitch = lastRightSwitch = true;
        lastLeftSpeed = lastRightSpeed = 0;
    }

    @Override
    protected void execute() {

        // get percent
        if(oi.getPilotDpad() == 0) percent = 0.2;
        else if(oi.getPilotDpad() == 180) percent = -0.2;
        else percent = 0;

        // handle limit switches
        boolean leftSwitch = climber.getLeftMagnetSwitch();
        boolean rightSwitch = climber.getRightMagnetSwitch();
        if(lastLeftSpeed > 0) {
            if(lastLeftSwitch != leftSwitch) {
                if(leftCanRetract) {
                    leftCanExtend = false;
                    leftCanRetract = true;
                } else {
                    leftCanExtend = true;
                    leftCanRetract = true;
                }
            }
            if(lastRightSwitch != rightSwitch) {
                if(rightCanRetract) {
                    rightCanExtend = false;
                    rightCanRetract = true;
                } else {
                    rightCanExtend = true;
                    rightCanRetract = true;
                }
            }
        } else if(lastLeftSpeed < 0) {
            if(lastLeftSwitch != leftSwitch) {
                if(leftCanExtend) {
                    leftCanExtend = true;
                    leftCanRetract = false;
                } else {
                    leftCanExtend = true;
                    leftCanRetract = true;
                }
            }
            if(lastRightSwitch != rightSwitch) {
                if(rightCanExtend) {
                    rightCanExtend = true;
                    rightCanRetract = false;
                } else {
                    rightCanExtend = true;
                    rightCanRetract = true;
                }
            }
        }

        // determine output values
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

        // save values for next loop
        if(leftPercent != 0) lastLeftSpeed = leftPercent;
        if(rightPercent != 0) lastRightSpeed = rightPercent;
        lastLeftSwitch = leftSwitch;
        lastRightSwitch = rightSwitch;

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
