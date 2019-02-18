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

    public ManualClimb() {
        requires(climber);
    }

    @Override
    protected void initialize() {
    }

    @Override
    protected void execute() {
        double pilotDpad = oi.getPilotDpad();
        if(pilotDpad == -1.0) climber.set(0);
        else if(pilotDpad == 0) climber.set(SPEED);
        else if(pilotDpad == 180) climber.set(-SPEED);
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
