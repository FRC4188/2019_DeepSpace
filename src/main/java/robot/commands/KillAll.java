package robot.commands;

import robot.Robot;
import edu.wpi.first.wpilibj.command.Command;

public class KillAll extends Command {
  
    public KillAll() {
        requires(Robot.drivetrain);
        requires(Robot.limelight);
    }

    @Override
    protected void initialize() {
    }

    @Override
    protected void execute() {
    }

    @Override
    protected boolean isFinished() {
        return true;
    }

    @Override
    protected void end() {
    }

    @Override
    protected void interrupted() {
    }
}
