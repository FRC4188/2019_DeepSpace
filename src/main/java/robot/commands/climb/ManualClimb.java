package robot.commands.climb;

import robot.OI;
import robot.Robot;
import robot.subsystems.Climber;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Runs climber motors at a given percent.
 *  Positive percent extends. */
public class ManualClimb extends Command {

    OI oi = Robot.oi;
    Climber climber = Robot.climber;

    double percent;
    boolean leftCanExtend, rightCanExtend, leftCanRetract, rightCanRetract;
    boolean lastLeftSwitch, lastRightSwitch;
    double lastLeftSpeed, lastRightSpeed;
    public double brownoutVariable;


    public ManualClimb(double percent) {
        requires(climber);
        this.percent = percent;
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

        // handle limit switches
        boolean leftSwitch = climber.getLeftMagnetSwitch();
        boolean rightSwitch = climber.getRightMagnetSwitch();
        if(lastLeftSwitch) {
            if(!leftSwitch) {
                if(lastLeftSpeed > 0) leftCanExtend = !leftCanExtend;
                if(lastLeftSpeed < 0) leftCanRetract = !leftCanRetract;
            }
        }
        if(lastRightSwitch) {
            if(!rightSwitch) {
                if(lastRightSpeed > 0) leftCanExtend = !leftCanExtend;
                if(lastRightSpeed < 0) leftCanRetract = !leftCanRetract;
            }
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
