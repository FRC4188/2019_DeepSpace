package robot.commands.drive;

import robot.Robot;
import robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.command.Command;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.followers.EncoderFollower;

/** Follows given path of waypoints using Pathfinder library.
 *  If isReversed == true, the path will run backwards. */
public class FollowPath extends Command {

    boolean isReversed;
    Waypoint[] points;
    EncoderFollower[] followers;
    Drivetrain drivetrain = Robot.drivetrain;

    public FollowPath(Waypoint[] points, boolean isReversed) {
        requires(drivetrain);
        this.points = points;
        this.isReversed = isReversed;
    }

    @Override
    protected void initialize() {
        followers = drivetrain.getEncoderFollowers(points);
    }

    @Override
    protected void execute() {
        drivetrain.followPath(followers, isReversed);
    }

    @Override
    protected boolean isFinished() {
        return drivetrain.isPathFinished();
    }

    @Override
    protected void end() {
        drivetrain.tank(0, 0, 0);
    }

    @Override
    protected void interrupted() {
    }

}
