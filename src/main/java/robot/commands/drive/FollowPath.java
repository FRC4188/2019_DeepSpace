package robot.commands.drive;

import robot.Robot;
import robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.command.Command;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.followers.EncoderFollower;
import jaci.pathfinder.modifiers.TankModifier;

/** Follows given path of waypoints using Pathfinder library.
 *  If isReversed == true, the path will run backwards. */
public class FollowPath extends Command {

    boolean isReversed;
    Waypoint[] points;
    EncoderFollower leftFollower, rightFollower;
    Drivetrain drivetrain = Robot.drivetrain;

    final double kP = 0.1;
    final double kI = 0;
    final double kD = 0;
    final double kV = 1.0 / drivetrain.MAX_VELOCITY;
    final double kA = 0;

    /** Follows path from given waypoints. isReversed causes the path
     * to be followed in reverse. */
    public FollowPath(Waypoint[] points, boolean isReversed) {
        requires(drivetrain);
        this.points = points;
        this.isReversed = isReversed;
    }

    @Override
    protected void initialize() {

        // create trajectory config
        Trajectory.Config config = new Trajectory.Config(
                Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_FAST, drivetrain.DELTA_T,
                drivetrain.MAX_VELOCITY, drivetrain.MAX_ACCELERATION, drivetrain.MAX_JERK);

        // generate trajectory + create followers
        Trajectory trajectory = Pathfinder.generate(points, config);
        TankModifier modifier = new TankModifier(trajectory).modify(drivetrain.WHEELBASE_WIDTH);
        leftFollower = new EncoderFollower(modifier.getLeftTrajectory());
        rightFollower = new EncoderFollower(modifier.getRightTrajectory());

        // follower config
        leftFollower.configureEncoder((int) drivetrain.getRawLeftPosition(),
                (int) drivetrain.TICKS_PER_REV, drivetrain.WHEEL_DIAMETER);
        rightFollower.configureEncoder((int) drivetrain.getRawRightPosition(),
                (int) drivetrain.TICKS_PER_REV, drivetrain.WHEEL_DIAMETER);
        leftFollower.configurePIDVA(kP, kI, kD, kV, kA);
        rightFollower.configurePIDVA(kP, kI, kD, kV, kA);

    }

    @Override
    protected void execute() {

        // invert drivetrain if needed
        if(isReversed) {
            drivetrain.setInverted(true);
        }

        // get motor setpoints
        double l = leftFollower.calculate((int) drivetrain.getRawLeftPosition());
        double r = rightFollower.calculate((int) drivetrain.getRawRightPosition());

        // turn control loop (kP from 254 presentation)
        double gyroHeading = drivetrain.getGyroAngle();
        double desiredHeading = Pathfinder.r2d(leftFollower.getHeading());
        double angleDifference = Pathfinder.boundHalfDegrees(desiredHeading - gyroHeading);
        double turn = 0.8 * (1.0/80.0) * angleDifference;
        System.out.println("angleDiff: " + angleDifference + " turn val: " + turn);

        // use output
        drivetrain.tank(l + turn, r - turn, 1.0);

    }

    @Override
    protected boolean isFinished() {
        return leftFollower.isFinished() && rightFollower.isFinished();
    }

    @Override
    protected void end() {
        drivetrain.tank(0, 0, 0);
        drivetrain.setInverted(false);
    }

    @Override
    protected void interrupted() {
        end();
    }

}
