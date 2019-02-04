package robot.utils;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;

/** Kills all currently running processes on the robot. */
public class KillAll extends Command {

    public KillAll() {
    }

    @Override
    protected void initialize() {
    }

    @Override
    protected void execute() {
        Scheduler.getInstance().removeAll();
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
