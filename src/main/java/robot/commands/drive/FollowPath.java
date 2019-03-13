package robot.commands.drive;

import robot.Robot;
import robot.subsystems.Drivetrain;
import robot.subsystems.LimeLight;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.PathfinderFRC;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.followers.DistanceFollower;
import jaci.pathfinder.followers.EncoderFollower;
import jaci.pathfinder.modifiers.TankModifier;

/** Follows given path of waypoints using Pathfinder library.
 *  If isReversed == true, the path will run backwards. */
public class FollowPath extends Command {

    public enum Path {

        TO_PERPENDICULAR(null), TEST("test");

        private final String file;
        Path(String file) {
            this.file = file;
        }

        public String getFile() {
            return file;
        }

    }

    Drivetrain drivetrain = Robot.drivetrain;
    LimeLight limelight = Robot.limelight;

    boolean isReversed, fromFile;
    double initialLeftDist, initialRightDist;
    Waypoint[] points;
    DistanceFollower leftFollower, rightFollower;
    Path path;

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
        this.fromFile = false;
    }

    public FollowPath(Path path, boolean isReversed) {
        requires(drivetrain);
        this.isReversed = isReversed;
        this.path = path;
        if(path == Path.TO_PERPENDICULAR) this.fromFile = false;
        else this.fromFile = true;
    }

    @Override
    protected void initialize() {

        // initial vals
        initialLeftDist = drivetrain.getLeftPosition();
        initialRightDist = drivetrain.getRightPosition();

        // create waypoints if going to perpendicular
        if(path == Path.TO_PERPENDICULAR) {

            // get necessary info from camera
            double currentAngle = Math.toRadians(drivetrain.getGyroAngle());
            double turnAngle = Math.toRadians(limelight.solvePerpendicular()[0]);
            double driveDist = limelight.solvePerpendicular()[1];
            double x = driveDist * Math.cos(turnAngle);
            double y = driveDist * Math.sin(turnAngle);
            double targetAngle = Math.toRadians(limelight.solvePerpendicular()[2]);

            SmartDashboard.putNumber("x drive", x);
            SmartDashboard.putNumber("y drive", y);
            SmartDashboard.putNumber("target angle", targetAngle);

            // create points
            points = new Waypoint[] {
                new Waypoint(0, 0, currentAngle),
                new Waypoint(x, y, targetAngle)
            };

        }

        // generate points if not from file
        if(!fromFile) {

            // generate trajectory
            Trajectory.Config config = new Trajectory.Config(
                    Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_FAST, drivetrain.DELTA_T,
                    drivetrain.MAX_VELOCITY, drivetrain.MAX_ACCELERATION, drivetrain.MAX_JERK);
            Trajectory trajectory = Pathfinder.generate(points, config);

            // create followers
            TankModifier modifier = new TankModifier(trajectory).modify(drivetrain.WHEELBASE_WIDTH);
            leftFollower = new DistanceFollower(modifier.getLeftTrajectory());
            rightFollower = new DistanceFollower(modifier.getRightTrajectory());

        } else {

            // get left and right trajectories
            Trajectory leftTrajectory = PathfinderFRC.getTrajectory(path.getFile() + ".left");
            Trajectory rightTrajectory = PathfinderFRC.getTrajectory(path.getFile() + ".right");

            // create followers
            leftFollower = new DistanceFollower(leftTrajectory);
            rightFollower = new DistanceFollower(rightTrajectory);

        }

        // follower config
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
        double l = leftFollower.calculate(drivetrain.getLeftPosition() - initialLeftDist);
        double r = rightFollower.calculate(drivetrain.getRightPosition() - initialRightDist);

        // turn control loop (kP from 254 presentation)
        double gyroHeading = drivetrain.getGyroAngle();
        double desiredHeading = Pathfinder.r2d(leftFollower.getHeading());
        double angleDifference = Pathfinder.boundHalfDegrees(desiredHeading - gyroHeading);
        double turn = 0.5 * (1.0/80.0) * angleDifference;
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
