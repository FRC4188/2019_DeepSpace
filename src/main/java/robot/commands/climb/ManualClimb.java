package robot.commands.climb;

import robot.OI;
import robot.Robot;
import robot.subsystems.Climber;
import edu.wpi.first.wpilibj.command.Command;

/** Runs climber motors at a given speed. Direction defined by
 *  pilot Dpad (up to retract, down to extend.)*/
public class ManualClimb extends Command {

    OI oi = Robot.oi;
    Climber climber = Robot.climber;

    final double SPEED = 0.7;
    boolean leftCanExtend, rightCanExtend, leftCanRetract, rightCanRetract;
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

        // direction based on Dpad
        double pilotDpad = oi.getPilotDpad();
        double leftPercent = 0;
        double rightPercent = 0;
        if(pilotDpad == 0) {
            if(leftCanExtend) leftPercent = SPEED;
            if(rightCanExtend) rightPercent = SPEED;
        } else if(pilotDpad == 180) {
            if(leftCanRetract) leftPercent = -SPEED;
            if(rightCanRetract) rightPercent = -SPEED;
        }

        // command motor output
        climber.setLeft(leftPercent);
        climber.setRight(rightPercent);

        // save values for next loop
        lastLeftSpeed = leftPercent;
        lastRightSpeed = rightPercent;
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
