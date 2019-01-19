package robot.commands.drive;

import robot.Robot;
import robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/** Follows line of reflective tape using photo sensors
 *  until robot hits wall. */
public class FollowLine extends Command {

    Timer timer = new Timer();
    Drivetrain drivetrain = Robot.drivetrain;

    public FollowLine() {
        requires(Robot.drivetrain);
        setTimeout(10);
    }

    @Override
    protected void initialize() {
        Robot.drivetrain.resetFollowLine();
        timer.start();
    }

    @Override
    protected void execute() {
        Robot.drivetrain.followLine();
    }

    @Override
    protected boolean isFinished() {
        // end if motor current spikes (hit wall) or on timeout
        // timer allows it to ignore initial current spike as motors are ramped up
        return false;
    }

    @Override
    protected void end() {
        timer.stop();
        Robot.drivetrain.tank(0, 0, 0);
        Robot.drivetrain.resetFollowLine();
    }

    @Override
    protected void interrupted() {
        end();
    }

}
