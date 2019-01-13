package robot.commands.drive;

import edu.wpi.first.wpilibj.command.Command;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.followers.EncoderFollower;
import robot.Robot;

/** Follows given path of waypoints using Pathfinder library.
 *  If isReversed == true, the path will run backwards. */
public class FollowPath extends Command {

    boolean isReversed;
    Waypoint[] points;
    EncoderFollower[] followers;

    public FollowPath(Waypoint[] points, boolean isReversed) {
        requires(Robot.drivetrain);
        this.points = points;
        this.isReversed = isReversed;
    }

    @Override
    protected void initialize() {
        followers = Robot.drivetrain.getEncoderFollowers(points);
    }

    @Override
    protected void execute() {
        Robot.drivetrain.followPath(followers, isReversed);
    }

    @Override
    protected boolean isFinished() {
        return Robot.drivetrain.isPathFinished();
    }

    @Override
    protected void end() {
        Robot.drivetrain.tank(0, 0, 0);
    }

    @Override
    protected void interrupted() {
    }

}
