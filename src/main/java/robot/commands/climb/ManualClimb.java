package robot.commands.climb;

import robot.OI;
import robot.Robot;
import robot.subsystems.Arm;
import robot.subsystems.Climber;
import robot.subsystems.Elevator;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;

public class ManualClimb extends Command {

    OI oi = Robot.oi;
    Climber climber = Robot.climber;

    public ManualClimb() {
        requires(climber);
    }

    @Override
    protected void initialize() {
    }

    @Override
    protected void execute() {
        climber.set(1.0);
        climber.set(-1.0);
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
