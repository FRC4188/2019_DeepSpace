package robot.commands.drive;

import edu.wpi.first.wpilibj.command.Command;
import robot.Robot;

public class FollowLine extends Command {

    public FollowLine() {
        requires(Robot.m_drivetrain);
    }

    @Override
    protected void initialize() {
    }

    @Override
    protected void execute() {
        Robot.m_drivetrain.followLine();
    }

    @Override
    protected boolean isFinished() {
        return false;
    }

    @Override
    protected void end() {
        Robot.m_drivetrain.resetFollowLine();
    }

    @Override
    protected void interrupted() {
        end();
    }

}
